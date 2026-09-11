#!/usr/bin/env python3
import os
import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    h0, w0 = im.shape[:2]
    nw, nh = new_shape
    r = min(nw / w0, nh / h0)
    w1, h1 = int(round(w0 * r)), int(round(h0 * r))
    im = cv2.resize(im, (w1, h1), interpolation=cv2.INTER_LINEAR)
    dw, dh = (nw - w1) / 2.0, (nh - h1) / 2.0
    left, top = int(round(dw - 0.1)), int(round(dh - 0.1))
    right, bottom = int(round(dw + 0.1)), int(round(dh + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, left, top


def nms(boxes, scores, iou_thres):
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T.astype(np.float32, copy=False)
    scores = scores.astype(np.float32, copy=False)
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order, keep = scores.argsort()[::-1], []
    while order.size:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1, yy1 = np.maximum(x1[i], x1[rest]), np.maximum(y1[i], y1[rest])
        xx2, yy2 = np.minimum(x2[i], x2[rest]), np.minimum(y2[i], y2[rest])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-6)
        order = rest[iou <= iou_thres]
    return keep


class YoloPersonPercentNode(Node):
    def __init__(self):
        super().__init__("yolo_person_percent_node")

        for name, default in [
            ("image_topic", "/camera/camera/color/image_raw"),
            ("model", "/home/yolo11n.onnx"),
            ("conf", 0.25),
            ("iou", 0.45),
            ("intra_op_num_threads", 2),
            ("inter_op_num_threads", 1),
            ("detections_topic", "/person_detections"),
            ("percent_topic", "/PT/percent"),
            ("no_person_percent", 0.0),
            ("publish_debug_image", True),
            ("debug_image_topic", "/person_debug_image"),
            ("show_window", False),
            ("window_name", "Person Detection + Max Percent"),
        ]:
            self.declare_parameter(name, default)

        p = lambda n: self.get_parameter(n).value
        self.image_topic = str(p("image_topic"))
        self.model_path = str(p("model"))
        self.conf_thres = float(p("conf"))
        self.iou_thres = float(p("iou"))
        self.intra_threads = max(1, int(p("intra_op_num_threads")))
        self.inter_threads = max(1, int(p("inter_op_num_threads")))
        self.detections_topic = str(p("detections_topic"))
        self.percent_topic = str(p("percent_topic"))
        self.no_person_percent = float(p("no_person_percent"))
        self.publish_debug = bool(p("publish_debug_image"))
        self.debug_image_topic = str(p("debug_image_topic"))
        self.show_window = bool(p("show_window"))
        self.window_name = str(p("window_name"))

        self.provider_mode = "cpu"
        self.execution_mode = "sequential"
        self.bridge = CvBridge()
        self.outdata = 0.0
        self._logged_output_shape = False

        if not os.path.isfile(self.model_path):
            self.get_logger().fatal(f"Model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.session = self._build_ort_session()
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.in_h = int(inp.shape[2]) if isinstance(inp.shape[2], int) else 640
        self.in_w = int(inp.shape[3]) if isinstance(inp.shape[3], int) else 640

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.pub_percent = self.create_publisher(Float32, self.percent_topic, 10)
        self.pub_dbg = self.create_publisher(Image, self.debug_image_topic, 10) if self.publish_debug else None
        self.create_timer(0.2, lambda: self.get_logger().info(f"{self.outdata}"))

        if self.show_window:
            if os.environ.get("DISPLAY", ""):
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            else:
                self.get_logger().warn("show_window=true nhưng DISPLAY chưa có, tự tắt window.")
                self.show_window = False

        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Input size: {self.in_w}x{self.in_h}")
        self.get_logger().info(f"Available providers: {ort.get_available_providers()}")
        self.get_logger().info(f"Using providers: {self.session.get_providers()}")
        self.get_logger().info(f"Provider mode: {self.provider_mode}")
        self.get_logger().info(f"Execution mode: {self.execution_mode}")

    def _build_ort_session(self):
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.intra_op_num_threads = self.intra_threads
        so.inter_op_num_threads = self.inter_threads
        so.enable_cpu_mem_arena = True
        so.log_severity_level = 3
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        return ort.InferenceSession(self.model_path, sess_options=so, providers=["CPUExecutionProvider"])

    def preprocess(self, frame):
        img, r, px, py = letterbox(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (self.in_w, self.in_h))
        img = np.ascontiguousarray(np.transpose(img.astype(np.float32) / 255.0, (2, 0, 1))[None], dtype=np.float32)
        return img, r, px, py

    def postprocess(self, outputs, ow, oh, r, px, py):
        if not outputs:
            return []
        out = np.array(outputs[0])
        while out.ndim > 2 and out.shape[0] == 1:
            out = out[0]
        if not self._logged_output_shape:
            self.get_logger().info(f"ONNX output shape: {tuple(out.shape)}")
            self._logged_output_shape = True

        if out.ndim == 2 and (out.shape[0] in (84, 85) or out.shape[1] in (84, 85)):
            pred = out.T if out.shape[0] in (84, 85) else out
            if pred.shape[1] < 6:
                return []
            boxes = pred[:, :4].astype(np.float32, copy=False)
            if pred.shape[1] == 84:
                cls = pred[:, 4:].astype(np.float32, copy=False)
                cls_ids = np.argmax(cls, axis=1)
                scores = cls[np.arange(len(cls)), cls_ids]
            else:
                obj = pred[:, 4].astype(np.float32, copy=False)
                cls = pred[:, 5:].astype(np.float32, copy=False)
                cls_ids = np.argmax(cls, axis=1)
                scores = obj * cls[np.arange(len(cls)), cls_ids]

            keep = (cls_ids == 0) & (scores >= self.conf_thres) & np.isfinite(scores)
            if not np.any(keep):
                return []
            boxes, scores = boxes[keep], scores[keep]
            cx, cy, w, h = boxes.T
            x1, y1, x2, y2 = (cx - w / 2 - px) / r, (cy - h / 2 - py) / r, (cx + w / 2 - px) / r, (cy + h / 2 - py) / r
            x1, y1 = np.clip(x1, 0, ow - 1), np.clip(y1, 0, oh - 1)
            x2, y2 = np.clip(x2, 0, ow - 1), np.clip(y2, 0, oh - 1)
            valid = (x2 - x1 > 2) & (y2 - y1 > 2) & np.isfinite(x1) & np.isfinite(y1) & np.isfinite(x2) & np.isfinite(y2)
            if not np.any(valid):
                return []
            boxes = np.stack([x1, y1, x2, y2], axis=1)[valid]
            scores = scores[valid]
            return [(float(*b[:1]), float(b[1]), float(b[2]), float(b[3]), float(scores[i])) for i, b in [(i, boxes[i]) for i in nms(boxes, scores, self.iou_thres)]]

        if out.ndim == 2 and 6 <= out.shape[1] <= 7:
            dets = []
            for x1, y1, x2, y2, score, cls_id, *_ in out:
                if int(cls_id) != 0 or float(score) < self.conf_thres:
                    continue
                x1, y1, x2, y2 = (x1 - px) / r, (y1 - py) / r, (x2 - px) / r, (y2 - py) / r
                x1, y1 = float(np.clip(x1, 0, ow - 1)), float(np.clip(y1, 0, oh - 1))
                x2, y2 = float(np.clip(x2, 0, ow - 1)), float(np.clip(y2, 0, oh - 1))
                if x2 - x1 > 2 and y2 - y1 > 2:
                    dets.append((x1, y1, x2, y2, float(score)))
            return dets
        return []

    def make_detection(self, header, x1, y1, x2, y2, score):
        det = Detection2D()
        det.header = header
        bbox = BoundingBox2D()
        bbox.center.position.x = float((x1 + x2) / 2)
        bbox.center.position.y = float((y1 + y2) / 2)
        bbox.center.theta = 0.0
        bbox.size_x = max(0.0, float(x2 - x1))
        bbox.size_y = max(0.0, float(y2 - y1))
        det.bbox = bbox
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "person"
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)
        return det

    def image_callback(self, msg):
        try:
            frame = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
            h, w = frame.shape[:2]
            img, r, px, py = self.preprocess(frame)
            dets = self.postprocess(self.session.run(None, {self.input_name: img}), w, h, r, px, py)
        except Exception as e:
            self.get_logger().error(f"inference error: {e}")
            return

        det_array = Detection2DArray()
        det_array.header = msg.header
        debug = frame.copy()
        img_area, max_percent = float(w * h) if w > 0 and h > 0 else 0.0, None

        for x1, y1, x2, y2, score in dets:
            det_array.detections.append(self.make_detection(msg.header, x1, y1, x2, y2, score))
            area_percent = max(0.0, min(100.0, ((x2 - x1) * (y2 - y1) / img_area) * 100.0)) if img_area > 0 else 0.0
            max_percent = area_percent if max_percent is None else max(max_percent, area_percent)

            if self.show_window or self.publish_debug:
                p1, p2 = (max(0, min(int(x1), w - 1)), max(0, min(int(y1), h - 1))), (max(0, min(int(x2), w - 1)), max(0, min(int(y2), h - 1)))
                cv2.rectangle(debug, p1, p2, (0, 255, 0), 2)
                cv2.putText(debug, f"person {score:.2f} | {area_percent:.2f}%", (p1[0], max(0, p1[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.pub_det.publish(det_array)
        out = Float32()
        out.data = float(self.no_person_percent if max_percent is None else max_percent)
        self.pub_percent.publish(out)
        self.outdata = out.data

        if self.show_window or self.publish_debug:
            cv2.putText(debug, f"Max Percent: {out.data:.2f}%", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        if self.publish_debug:
            try:
                dbg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                dbg.header = msg.header
                self.pub_dbg.publish(dbg)
            except Exception as e:
                self.get_logger().error(f"debug publish error: {e}")

        if self.show_window:
            cv2.imshow(self.window_name, debug)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                self.get_logger().info("Pressed 'q' -> shutting down node")
                rclpy.shutdown()

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonPercentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
