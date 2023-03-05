import cfg
import argparse


def get_args_parser():
    parser = argparse.ArgumentParser('Setting EyeCar args', add_help=False)
    parser.add_argument(
        "--video_source_path",
        type=str,
        default=cfg.DEFAULT_VIDEO_SOURCE_PATH,
    )
    parser.add_argument(
        "--video_source_file",
        type=str,
        default=cfg.DEFAULT_VIDEO_SOURCE_FILE,
    )
    parser.add_argument(
        "--img_source_folder_path",
        type=str,
        default=cfg.DEFAULT_IMG_SOURCE_FOLDER_PATH,
    )
    parser.add_argument(
        "--model_folder",
        type=str,
        default=cfg.DEFAULT_MODEL_FOLDER,
    )

    return parser
