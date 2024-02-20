from typing import Tuple
import os


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    print("==========================",os.getcwd()) # /tmp => which container?
#    filename = "/code/object-detection/assets/DT_TOKEN"
#    file = open(filename, "r")
#    dt_token = file.read().rstrip()
#    file.close()
    return "dt1-..." # edit before submission


def MODEL_NAME() -> str:
    # TODO: change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5n"
    return "yolov5n_800imgs_100E_best"


def NUMBER_FRAMES_SKIPPED() -> int:
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 5


def filter_by_classes(pred_class: int) -> bool:
    """
    Remember the class IDs:

        | Object    | ID    |
        | ---       | ---   |
        | Duckie    | 0     |
        | Cone      | 1     |
        | Truck     | 2     |
        | Bus       | 3     |


    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    if pred_class == 0 :
        return True
    else:
        return False


def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    if score > 0.7:
        return True
    else:
        return False


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    if height > 70 or width > 70:
        return True
    else:
        return False



