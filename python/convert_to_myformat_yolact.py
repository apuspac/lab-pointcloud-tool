"""

COCO anotationからexportしたjsonファイルをこのプログラムで使用する形式に変換するプログラム

Usage:
python convert_to_myformat.py {input_directory_path} {save_path}
python convert_to_mayformat.py json_data out

"""
# standard Library
import sys
import os
import json
import getopt
import argparse
import pathlib
import pprint

class COCO_Anotation_convert():
    def __init__(self) -> None:
        pass


    def load_anotation(self, input_path: str):
        """Load annotation data from json file

        Args:
            input_path (str): Name of the directory containing the json file

        Returns:
            jsonfile: Loaded json file
        """
        json_open = open(input_path, 'r')
        json_load = json.load(json_open)

        return json_load

    def save_results(self, results, output_name, save_path):
        """Save converted json file

        Args:
            results (json): JSON file to be saved
            output_name (string): Name of the save
            save_path (string): Path to the save diarectory
        """
        # Save the converted JSON file
        save_path = os.path.join(save_path, output_name)
        with open(save_path, 'w') as outfile:
            json.dump(results, outfile, indent=4)

    def convert_json_format(self, json_load):
        """Convert the JSON file from COCO annotation to my program format.

        Args:
            json_load (json): loaded json file

        Returns:
            json: Converted json file
        """
        # Load the filename and filepath
        input_path = json_load['images'][0]['path']
        filepath = json_load['images'][0]['path']
        filename = json_load['images'][0]['file_name']

        categories = {}
        bbox_info = []
        mask_info = []

        # Create a dictionary of categories
        for category in json_load['categories']:
            categories[category['id']] = category['name']

        for one_annotation in json_load['annotations']:
            if categories[one_annotation['category_id']] == 'pipe':
                # Get mask data
                segmentation = one_annotation['segmentation']
                contour_points = []

                for one_seg in segmentation:
                    it = iter(one_seg)
                    for x, y in zip(it, it):
                        # Convert to integer and wrap in nested list for contour format
                        contour_points.append([[int(x), int(y)]])

                confidence = one_annotation.get('score', 0)
                mask_info.append({
                    'class_name': 'pipe',
                    'contour': contour_points,
                    'confidence': confidence
                })
            else:
                # Get bbox data
                bbox = one_annotation['bbox']
                xmin = int(bbox[0])
                ymin = int(bbox[1])
                xmax = int(bbox[0] + bbox[2])
                ymax = int(bbox[1] + bbox[3])

                confidence = 0
                classid = one_annotation['category_id']
                name = categories[one_annotation['category_id']]
                bbox_info.append({
                    'xmin': xmin,
                    'ymin': ymin,
                    'xmax': xmax,
                    'ymax': ymax,
                    'confidence': confidence,
                    'class': classid,
                    'name': name
                })

        merged_data = {
            'file_path': filepath,
            'file_name': filename,
            'bbox_info': bbox_info,
            'mask_info': mask_info
        }
        merged_data_list = [merged_data]
        convert_format = {'input_path': input_path, 'merged_data': merged_data_list}

        return convert_format

def parse_args(argv):
    target_folder_path = ''

    if len(sys.argv) < 3:
        raise ValueError("Missing arguments")
        sys.exit(2)
    else :
        target_folder_path = sys.argv[1]
        save_path = sys.argv[2]

    return target_folder_path, save_path

if __name__ == "__main__":
    convert_annotation = COCO_Anotation_convert()

    # Path of the directory containing JSON files that will be converted to my program format
    input_directory_path, save_path = parse_args(sys.argv)

    convert_path = []

    # Convert the JSON fiel from exported COCO anotation
    for filename in os.listdir(input_directory_path):
        json_file_path = os.path.join(input_directory_path, filename)

        load_json = convert_annotation.load_anotation(json_file_path)
        convert_json = convert_annotation.convert_json_format(load_json)
        convert_annotation.save_results(convert_json, filename, save_path)


