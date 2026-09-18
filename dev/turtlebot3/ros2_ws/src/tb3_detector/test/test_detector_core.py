import unittest

from tb3_detector.detector_core import DetectorCore, parse_locate_anything_boxes


class ParseLocateAnythingBoxesTests(unittest.TestCase):
    def test_parses_labels_and_scales_normalized_coordinates(self):
        answer = (
            "<ref>person</ref><box><100><200><600><900></box>"
            "<ref>bench</ref><box><0><500><1000><1000></box>"
        )

        detections = parse_locate_anything_boxes(answer, 640, 480, ["bench", "person"])

        self.assertEqual([d["label"] for d in detections], ["person", "bench"])
        self.assertEqual(detections[0]["bbox_xyxy"], [64.0, 96.0, 384.0, 432.0])
        self.assertEqual(detections[1]["bbox_xyxy"], [0.0, 240.0, 640.0, 480.0])
        self.assertTrue(all(d["conf"] == 1.0 for d in detections))
        self.assertTrue(all(d["track_id"] is None for d in detections))

    def test_reuses_single_requested_label_when_ref_tag_is_absent(self):
        detections = parse_locate_anything_boxes(
            "<box><10><20><30><40></box>", 1000, 1000, ["stop sign"]
        )

        self.assertEqual(detections[0]["label"], "stop sign")

    def test_ignores_unlabelled_and_degenerate_boxes(self):
        answer = (
            "<box><10><20><30><40></box>"
            "<ref>person</ref><box><100><100><100><300></box>"
        )

        self.assertEqual(
            parse_locate_anything_boxes(answer, 640, 480, ["bench", "person"]), []
        )

    def test_ignores_boxes_for_unrequested_labels(self):
        answer = "<ref><962></ref><box><100><200><300><400></box>"

        self.assertEqual(
            parse_locate_anything_boxes(answer, 640, 480, ["bench", "person"]), []
        )

    def test_rejects_an_empty_query_list(self):
        with self.assertRaisesRegex(ValueError, "at least one class"):
            DetectorCore(class_filter=[])


if __name__ == "__main__":
    unittest.main()
