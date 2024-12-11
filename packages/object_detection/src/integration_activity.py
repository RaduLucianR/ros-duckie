from typing import Tuple

def MODEL_NAME() -> str:
    # Model name
    return "ourdatasetWithDuckies"


def NUMBER_FRAMES_SKIPPED() -> int:
    # Number of frames to drop
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
        | Duckie            | 6     |


    Args:
        pred_class: the class of a prediction
    """

    return True


def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # print("score:", score)
    # If score > 0.75, send detection signal, else do not
    if score > 0.75:
        return True

    return False


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    #size of image 416x416
    # print("x:", str(abs(bbox[2] - bbox[0])))
    # print("y:", str(abs(bbox[1] - bbox[3])))

    # If box is too small or too large, ignore. Else, send detection signal. 
    if ((abs(bbox[2] - bbox[0]) < 35) or (abs(bbox[1]-bbox[3])) < 50):
       return False

    if ((abs(bbox[2] - bbox[0]) > 300) or (abs(bbox[1]-bbox[3])) > 300):
       return False
    
    return True
