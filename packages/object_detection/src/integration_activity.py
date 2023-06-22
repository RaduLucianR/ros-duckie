from typing import Tuple


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    dt_token = "dt1-3nT8KSoxVh4MnCdNw2Ch8NVrnJeQ3wLj14CryH3kh1g5RjA-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfbcNqTXWiUkMahkap2RM6k7mgx2wPKJ766"
    return dt_token


def MODEL_NAME() -> str:
    # TODO: change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5n"
    return "ourdatasetWithDuckies"


def NUMBER_FRAMES_SKIPPED() -> int:
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 2


def filter_by_classes(pred_class: int) -> bool:
    """
    Remember the class IDs:

        | Object            | ID    |
        | ---               | ---   |
        | Speed Limit 20    | 0     |
        | Speed Limit 30    | 1     |
        | Speed Limit 50    | 2     |
        | Stop             `| 3     |
        | Turn Right        | 4     |
        | Turn Left         | 5     |
        | Duckie         | 6     |


    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    if pred_class == 14:
        return True
    else:
        return True
    return True


def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    #print("score:", score)
    if score < 0.4:
        return False
    else:
        return True
    return True


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    #size of image 416x416
    #print("x: %d\n", (abs(bbox[2] - bbox[0])))
    #print("y: %d\n", (abs(bbox[1] - bbox[3])))
    # print("x:", str(abs(bbox[2] - bbox[0])))
    # print("y:", str(abs(bbox[1] - bbox[3])))
    #print("area of box:", str(abs(bbox[2] - bbox[0])*abs(bbox[1]-bbox[3])))
    #print("area of string: %d\n", (abs(bbox[2] - bbox[0])*abs(bbox[1]-bbox[3])))
    # if ((bbox[0] <= 416/3) or (bbox[2] >= 2*(416/3)) or ((abs(bbox[2] - bbox[0])*abs(bbox[1]-bbox[3])) <= 0.05*(416*416))):
    #     return False
    # else:
    #     return True 
    # if ((abs(bbox[2] - bbox[0])*abs(bbox[1]-bbox[3])) <= 0.2*(416*416)):
    #    return False
    # else:
    #    return True
    if ((abs(bbox[2] - bbox[0]) > 300) or (abs(bbox[1]-bbox[3])) > 300):
       return False
    else:
       return True
    return True
