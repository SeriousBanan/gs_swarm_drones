import os
import cv2
import time
import numpy as np
import tflite_runtime.interpreter as tflite


class Detector:

    def __init__(self, threshold):
        """Initialize tflite model."""
        self.interpreter = tflite.Interpreter(
            model_path="%s/model/model_v2_320_corr_exp.tflite" %
            (os.environ.get("DETECTOR_PATH")))
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.width = self.input_details[0]['shape'][1]
        self.height = self.input_details[0]['shape'][2]
        self.threshold = threshold

        if not os.path.exists("%s/images/" % os.environ.get("DETECTOR_PATH")):
            os.mkdir("%s/images/" % os.environ.get("DETECTOR_PATH"))
        if not os.path.exists("%s/images/detected/" % os.environ.get("DETECTOR_PATH")):
            os.mkdir("%s/images/detected/" % os.environ.get("DETECTOR_PATH"))
        if not os.path.exists("%s/images/raw/" % os.environ.get("DETECTOR_PATH")):
            os.mkdir("%s/images/raw/" % os.environ.get("DETECTOR_PATH"))

    def check_image_dirs(self):
        """Clean image folders based on threshold."""
        images_detected = os.listdir("%s/images/detected/" % os.environ.get("DETECTOR_PATH"))
        if len(images_detected) == 10:
            for image in images_detected:
                os.remove("%s/images/detected/%s" % (os.environ.get("DETECTOR_PATH"), image))
        images_raw = os.listdir("%s/images/raw/" % os.environ.get("DETECTOR_PATH"))
        if len(images_raw) == 30:
            for image in images_raw:
                os.remove("%s/images/raw/%s" % (os.environ.get("DETECTOR_PATH"), image))

    def draw_boxes(self, img, boxes, scores, img_height, img_width):
        """Draw box on the detected tank."""
        for i in range(len(boxes)):
            ymin = round(boxes[i][0] * img_height)
            xmin = round(boxes[i][1] * img_width)
            ymax = round(boxes[i][2] * img_height)
            xmax = round(boxes[i][3] * img_width)
            bbox_thick = int(0.6 * (len(img) + len(img[0])) / 600)
            c1, c2 = (int(xmin), int(ymin)), (int(xmax), int(ymax))
            cv2.rectangle(img, c1, c2, (10, 255, 0), 2)
            bbox_mess = "tank " + str(scores[i])
            t_size = cv2.getTextSize(bbox_mess, 0, 0.5, thickness=bbox_thick // 2)[0]
            cv2.rectangle(img, c1, (c1[0] + t_size[0], c1[1] - t_size[1] - 3), (10, 255, 0), -1)
            cv2.putText(img, bbox_mess, (c1[0], c1[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), bbox_thick // 2, lineType=cv2.LINE_AA)
        return img

    def detect(self, img):
        """Conduct detection for the given image."""
        if isinstance(img, list):
            return False, None
        self.check_image_dirs()
        input_mean, input_std = 127.5, 127.5
        raw_filename = int(time.time())
        cv2.imwrite("%s/images/raw/%d.jpg" % (os.environ.get("DETECTOR_PATH"), raw_filename), img)
        img_height, img_width, _ = img.shape
        image_resized = cv2.resize(img, (self.width, self.height))
        input_data = np.expand_dims((image_resized - input_mean) / input_std, axis=0).astype(np.float32)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        detected_boxes, detected_scores = [], []
        for i in range(len(scores)):
            if scores[i] > self.threshold:
                detected_boxes.append(boxes[i])
                detected_scores.append(scores[i])
        if len(detected_boxes) == 0:
            return False, None
        res = self.draw_boxes(img, detected_boxes, detected_scores, img_height, img_width)
        self.check_image_dirs()
        detected_filename = int(time.time())
        cv2.imwrite("%s/images/detected/%d.jpg" % (os.environ.get("DETECTOR_PATH"), detected_filename), res)
        return True, "%s/images/detected/%d.jpg" % (os.environ.get("DETECTOR_PATH"), detected_filename)
