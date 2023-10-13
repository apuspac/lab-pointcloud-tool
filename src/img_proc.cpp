/**
 * @file img_proc.cpp
 * @brief
 *
 *
 */

#include "img_proc.hpp"

void InstaImg::load_img(std::string img_path)
{
    img = cv::imread(img_path);
    if (img.empty())
    {
        std::cout << "Could not open or find the image" << std::endl;
        std::exit(-1);
    }
    else
    {
        std::cout << "Success load image: " << img_path << std::endl;
        name = img_path;
        height = img.cols;
        width = img.rows;
    }
}
