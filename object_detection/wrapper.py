""" Copyright 2023, Rayan Bahrami, 
    Safe Autonomous Systems Lab (SAS Lab)
    Stevens Institute of Technology
    See LICENSE file for the license information.

    credit: part of the code is adapted and modified from the following repository:
    https://github.com/mit-acl/yolov7_ros

    This module provides a wrapper for the YOLOv7 object detection model.
    It provides a method to detect objects in an image and draw bounding boxes, and adversarially perturb the bounding boxes
"""

import sys
import pathlib

import numpy as np
import torch
from copy import deepcopy

import random


# add yolov7 submodule to path
FILE_ABS_DIR = pathlib.Path(__file__).absolute().parent
YOLOV7_ROOT = (FILE_ABS_DIR / "yolov7").as_posix()
if YOLOV7_ROOT not in sys.path:
    sys.path.append(YOLOV7_ROOT)
    # print(f"YOLOV7_ROOT: {YOLOV7_ROOT}")

from utils.general import (
    check_img_size,
    scale_coords,
    non_max_suppression,
    xyxy2xywh,
    xywh2xyxy,
)

# from utils.datasets import letterbox
from models.experimental import attempt_load

# from utils.plots import plot_one_box
from utils.loss import ComputeLoss

import cv2


def letterbox(
    img,
    new_shape=(640, 640),
    color=(114, 114, 114),
    auto=True,
    scaleFill=False,
    scaleup=True,
    stride=32,
):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(
        img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )  # add border
    return img, ratio, (dw, dh)


# fmt: off
def plot_one_box(x, img, color=None, label=None, label2=None, line_thickness=3, loc="top"):
    # Plots one bounding box on image img_bgr
    tl = (line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1)  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]

        if loc == "bottem" and label2 is not None:
            t_size2 = cv2.getTextSize(label2, 0, fontScale=tl / 3, thickness=tf)[0]
            cv2.rectangle(
                img,
                (c1[0], c2[1]),
                (
                    # int(c2[0] + max(t_size[0], t_size2[0])),
                    int(c2[0] + max(t_size[0], t_size2[0]) * 0.8),
                    int(c2[1] + 2 + t_size[1] + t_size2[1] + 2),
                ),
                color,
                -1,
                cv2.LINE_AA,
            )  # filled
            cv2.putText(img, label, (c1[0], c2[1] + t_size[1]), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
            cv2.putText(img, label2, (c1[0], c2[1] + t_size[1] + 2 + t_size2[1]), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        else:
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 3), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


class YoloV7:
    def __init__(
        self,
        weights,
        conf_thresh: float = 0.5,
        iou_thresh: float = 0.45,
        img_size: int = 640,
        device: str = "cuda",
        bbox_plot_line_thickness=1,
    ):

        self.__conf_thresh = conf_thresh
        self.__iou_thresh = iou_thresh
        self.__device = device
        self.__weights = weights

        self.model = attempt_load(self.__weights, map_location=self.__device)
        self.__stride = int(self.model.stride.max())
        self.__img_size = check_img_size(img_size, s=self.__stride)
        self.__names = self.model.names  # list of class names

        self.__half = False

        # vizulization
        self.__colors = [
            [random.randint(0, 255) for _ in range(3)] for _ in self.__names
        ]
        self.__line_thickness = bbox_plot_line_thickness

        self.model_info()

    def model_info(self, verbose=False, img_size=640):
        # Model information. img_size may be int or list, i.e. img_size=640 or img_size=[640, 320]
        n_p = sum(x.numel() for x in self.model.parameters())  # number parameters
        n_g = sum(
            x.numel() for x in self.model.parameters() if x.requires_grad
        )  # number gradients
        if verbose:
            print(
                "%5s %40s %9s %12s %20s %10s %10s"
                % ("layer", "name", "gradient", "parameters", "shape", "mu", "sigma")
            )
            for i, (name, p) in enumerate(self.model.named_parameters()):
                name = name.replace("module_list.", "")
                print(
                    "%5g %40s %9s %12g %20s %10.3g %10.3g"
                    % (
                        i,
                        name,
                        p.requires_grad,
                        p.numel(),
                        list(p.shape),
                        p.mean(),
                        p.std(),
                    )
                )

        try:  # FLOPS
            from thop import profile

            stride = (
                max(int(self.model.stride.max()), 32)
                if hasattr(self.model, "stride")
                else 32
            )
            img = torch.zeros(
                (1, self.model.yaml.get("ch", 3), stride, stride),
                device=next(self.model.parameters()).device,
            )  # input
            flops = (
                profile(deepcopy(self.model), inputs=(img,), verbose=False)[0] / 1e9 * 2
            )  # stride GFLOPS
            img_size = (
                img_size if isinstance(img_size, list) else [img_size, img_size]
            )  # expand if int/float
            fs = ", %.1f GFLOPS" % (
                flops * img_size[0] / stride * img_size[1] / stride
            )  # 640x640 GFLOPS
        except (ImportError, Exception):
            fs = ""
        summary = (
            f"\N{rocket}\N{rocket}\N{rocket} Yolov7 Detector summary:\n"
            + f"Weights: {self.__weights}\n"
            + f"Confidence Threshold: {self.__conf_thresh}\n"
            + f"IOU Threshold: {self.__iou_thresh}\n"
            + f"{len(list(self.model.modules()))} layers, {n_p} parameters, {n_g} gradients{fs}\n"
            + f"Classes ({len(self.__names)}): {self.__names}"
        )
        print(summary)

    @torch.no_grad()
    def _inference(self, img: torch.Tensor, classes=None, multi_label=False):
        """
        :param img: tensor [c, h, w]
        :returns: tensor of shape [num_boxes, 6], where each item is represented as
            [x1, y1, x2, y2, confidence, class_id]
        """
        pred_results = self.model(img)[0]
        # the output, pred_results, follows the Detection class
        # here https://github.com/WongKinYiu/yolov7/blob/a207844b1ce82d204ab36d87d496728d3d2348e7/models/yolo.py#L23C10
        detections = non_max_suppression(
            pred_results,
            conf_thres=self.__conf_thresh,
            iou_thres=self.__iou_thresh,
            classes=classes,
            multi_label=multi_label,
        )

        if detections:
            detections = detections[0]

        return detections

    def _process_img(self, img0):

        # Padded resize (i.e., maintain aspect ratio)
        img = letterbox(img0, self.__img_size, stride=self.__stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        # processing -- cf. yolov7/detect.py
        img = torch.from_numpy(img).to(self.__device)
        img = img.half() if self.__half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return img

    def detect(
        self,
        img0,
        classes=None,
        multi_label=False,
        perturb_boxes=False,
        perturb_target_id=2,
        deviation_percentage=0.1,
        num_points=10,
    ):
        """
        Perform inference on an image to detect classes.

        Parameters
        ----------
        img0 : (h, w, c) np.array -- the input image
        classes : list of int -- classes to detect | None for all classes

        Returns
        -------
        dets : (n, 6) np.array -- n detections
                Each detection is 2d bbox xyxy, confidence, class
        dets_xywh : (n, 6) np.array -- n detections
                Each detection is 2d bbox xywh, confidence, class
        """

        # process the input image to make it appropriate for inference
        img = self._process_img(img0)

        # apply inference model to processed image
        dets = self._inference(img, classes=classes, multi_label=multi_label)

        # Rescale boxes from img_size to im0 size
        dets[:, :4] = scale_coords(img.shape[2:], dets[:, :4], img0.shape).round()

        if perturb_boxes:
            # Filter and select the relevant row
            dets = self._adv_perturb(
                dets,
                target_id=perturb_target_id,
                deviation_percentage=deviation_percentage,
                num_points=num_points,
                H=img0.shape[0],
                W=img0.shape[1],
                torch_seed=35,
            )

        dets_xywh = deepcopy(dets)
        dets_xywh[:, :4] = xyxy2xywh(dets_xywh[:, :4])

        return dets.cpu().detach().numpy(), dets_xywh.cpu().detach().numpy()

    def _adv_perturb(
        self,
        dets,
        target_id,
        deviation_percentage=0.15,
        num_points=10,
        H=480,
        W=640,
        torch_seed=35,
    ):
        d = dets[dets[:, -1] == target_id][:, :-1]
        device = d.device

        if len(d) == 0:
            return dets
        else:
            d = d[0]  # d is now a 1D tensor
            torch.manual_seed(torch_seed)  # for reproducibility

            # Calculate deviation ranges
            x1_range = deviation_percentage * d[0]
            y1_range = deviation_percentage * d[1]
            x2_range = deviation_percentage * d[2]
            y2_range = deviation_percentage * d[3]

            # Generate random deviations within the range [-deviation_range, deviation_range]
            x1_deviation = (
                torch.rand(num_points, device=device) * 2 * x1_range
            ) - x1_range
            y1_deviation = (
                torch.rand(num_points, device=device) * 2 * y1_range
            ) - y1_range
            x2_deviation = (
                torch.rand(num_points, device=device) * 2 * x2_range
            ) - x2_range
            y2_deviation = (
                torch.rand(num_points, device=device) * 2 * y2_range
            ) - y2_range

            # Calculate new points (x', y') and clip to ensure values are within the specified range
            x1_new = torch.clamp(d[0] + x1_deviation, 0, W)
            y1_new = torch.clamp(d[1] + y1_deviation, 0, H)
            x2_new = torch.clamp(d[2] + x2_deviation, 0, W)
            y2_new = torch.clamp(d[3] + y2_deviation, 0, H)

            # Pre-allocate the adversarial detections
            adv_dets = torch.empty((num_points, 6), dtype=torch.float32)

            adv_dets[:, 0] = x1_new
            adv_dets[:, 1] = y1_new
            adv_dets[:, 2] = x2_new
            adv_dets[:, 3] = y2_new

            adv_dets[:, 4] = min(d[4] * 1.1, 0.99)  # increase confidence by 10%
            adv_dets[:, 5] = target_id  # clas target
            adv_dets = adv_dets.to(device)

            # Concatenate tensors
            dets = torch.cat((dets, adv_dets), dim=0)
            return dets

    def draw_2d_bboxes(self, img, dets):
        for det in dets:
            *xyxy, conf, cls = det
            label = f"{self.__names[int(cls)]} {conf:.2f}"
            plot_one_box(
                xyxy,
                img,
                label=label,
                color=self.__colors[int(cls)],
                line_thickness=self.__line_thickness,
            )

    def draw_2d_bboxes_trk(self, img, dets, loc="bottem"):
        for det in dets:
            *xywh, cov_trace, trk_id, cls = det
            xyxy = xywh2xyxy(np.array([xywh])).squeeze()
            # label = f"track id: {trk_id} {self.__names[int(cls)]}\n cov_trace {cov_trace:.2f}"
            label = f"track id: {trk_id} {self.__names[int(cls)]}"
            label2 = f"cov_trace {cov_trace:.2f}"
            plot_one_box(
                xyxy,
                img,
                label=label,
                label2=label2,
                color=[35, 155, 86],  # self.__colors[int(cls)],
                line_thickness=self.__line_thickness,
                loc=loc,
            )


if __name__ == "__main__":
    import cv2 as cv
    import numpy as np

    weights = "yolov7-tiny-best.pt"
    img_name = "cam-test.png"

    weights_path = FILE_ABS_DIR / weights

    detector = YoloV7(
        weights_path.as_posix(),
        conf_thresh=0.25,
        iou_thresh=0.45,
        img_size=640,
        device="cuda",
    )

    img_path = FILE_ABS_DIR / img_name
    img = cv.imread(img_path.as_posix())
    image = img.copy()
    cv.imshow("img", img)
    cv.waitKey(1000)
    # print(f"img.shape: {img.shape}")  # (H, W, c)

    dets, dets_xywh = detector.detect(
        img,
        classes=None,
        multi_label=False,
        perturb_boxes=False,
        deviation_percentage=0.1,
        perturb_target_id=2,
    )
    print(f"dets (xywh):\n{dets_xywh}")
    detector.draw_2d_bboxes(img, dets)
    cv.imshow("img", img)
    cv.waitKey(5000)
