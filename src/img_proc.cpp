/**
 * @file img_proc.cpp
 * @brief
 *
 *
 */

#include "img_proc.hpp"
#include <utility>

/**
 * @brief img_pathから画像を読み込む
 *
 * @param img_path 画像の相対パス
 */
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
        // openCV 逆っぽいため こちらも反転しておく
        height = img.rows;
        width = img.cols;
    }
}

/**
 * @brief 画像を表示する
 *
 * @param window_name
 * @param scale resizeするときの倍率(今回は0.25がちょうどよい)
 */
void InstaImg::show(std::string window_name, double scale)
{
    cv::Mat output;
    cv::resize(img, output, cv::Size(), scale, scale);
    cv::imshow(window_name, output);
    cv::waitKey(0);
}

/**
 * @brief 画像を表示する
 *
 * @param window_name
 * @param scale resizeするときの倍率(今回は0.25がちょうどよい)
 * @param img 表示する画像
 */
void InstaImg::show(std::string window_name, double scale, const cv::Mat out_img)
{
    cv::Mat output;
    cv::resize(out_img, output, cv::Size(), scale, scale);
    cv::imshow(window_name, output);
    cv::waitKey(0);
}

/**
 * @brief cv::Matを0で初期化する
 *
 * @param _height
 * @param _width
 * @param color_type gray画像ならCV_8UC1
 */
void InstaImg::set_zero_imgMat(int _height, int _width, int color_type)
{
    img = cv::Mat::zeros(_height, _width, color_type);
    set_height(_height);
    set_width(_width);
}

/**
 * @brief 2値画像の指定したpixelを255にする
 *
 * @param u
 * @param v
 */
void InstaImg::set_pixel_255(int u, int v)
{
    if (img.type() != CV_8UC1)
    {
        std::cout << "img type is not CV_8UC1:  " << name << " " << img.type() << std::endl;
    }
    if (u > img.cols || v > img.rows)
    {
        std::cout << "u or v is over img size: " << u << " " << v << "img_size: " << img.cols << " " << img.rows << std::endl;
    }
    // if (u < 0 || v < 0)
    // {
    //     std::cout << "u or v is under 0: " << u << " " << v << std::endl;
    // }
    img.at<u_char>(v, u) = 255;
}

/**
 * @brief テスト画像 すいか
 */
void InstaImg::make_test_img_forEdge(int test_width, int test_height, int step, int band)
{
    set_zero_imgMat(test_height, test_width, CV_8UC1);

    int step_tmp = 0;
    int band_tmp = 0;
    for (int i = 0; i < test_width; i++)
    {
        for (int j = 0; j < test_height; j++)
        {
            if (step_tmp > step && band_tmp < band)
            {
                set_pixel_255(i, j);
            }
        }

        if (step_tmp == step && band_tmp == band)
        {
            step_tmp = 0;
            band_tmp = 0;
        }
        else if (step_tmp > step && band_tmp < band)
        {
            band_tmp++;
        }
        else
        {
            step_tmp++;
        }
    }
}

/**
 * @brief 縮小処理
 *
 * @param erosin_elem: 0: 長方形 1: 十字型 2: 楕円ぽい感じ
 * @param erosion_size  カーネルサイズは1で3*3 2で5*5 3で7*7
 */
void InstaImg::erosion(int erosion_elem, int erosion_size)
{
    int erosion_type = 0;
    if (erosion_elem == 0)
    {
        erosion_type = cv::MORPH_RECT;
    }
    else if (erosion_elem == 1)
    {
        erosion_type = cv::MORPH_CROSS;
    }
    else if (erosion_elem == 2)
    {
        erosion_type = cv::MORPH_ELLIPSE;
    }

    // カーネル作成
    // shape, kernel size, anchor
    // カーネルサイズは1で3*3 2で5*5 3で7*7
    // anchorは注目画素をどこにするか。指定しないとカーネル中心を注目画素にする
    cv::Mat element = cv::getStructuringElement(erosion_type,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));
    cv::erode(img, img, element);
    // show("Erosion Demo", 0.25);
}

/**
 * @brief 膨張処理
 *
 * @param dilation_elem : 0: 長方形 1: 十字型 2: 楕円ぽい感じ
 * @param dilation_size : カーネルサイズは1で3*3 2で5*5 3で7*7
 */
void InstaImg::dilation(int dilation_elem, int dilation_size)
{
    cv::Mat dilation_dst;
    int dilation_type = 0;
    if (dilation_elem == 0)
    {
        // 長方形(全部埋めてる)
        dilation_type = cv::MORPH_RECT;
    }
    else if (dilation_elem == 1)
    {
        // 十字型
        dilation_type = cv::MORPH_CROSS;
    }
    else if (dilation_elem == 2)
    {
        // 楕円ぽい感じ
        dilation_type = cv::MORPH_ELLIPSE;
    }

    // カーネル作成
    // shape, kernel size, anchor
    // カーネルサイズは1で3*3 2で5*5 3で7*7
    // anchorは注目画素をどこにするか。指定しないとカーネル中心を注目画素にする
    cv::Mat element = cv::getStructuringElement(
        dilation_type,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));

    cv::dilate(img, img, element);
    // show("dilation demo", 0.25);
}

/**
 * @brief ガウシアンフィルタを適用する
 *
 * @param kernel_size カーネルサイズ
 */
void InstaImg::gaussian_blur(int kernel_size)
{
    cv::GaussianBlur(img, img, cv::Size(kernel_size, kernel_size), 0, 0, cv::BORDER_DEFAULT);
}

/**
 * @brief canny法によってエッジ検出
 *
 * @param origin_img Canny法を適用する画像
 */
void EdgeImg::detect_edge_with_canny(const cv::Mat &origin_img)
{
    cv::Mat output, tmp;

    cv::Canny(origin_img, tmp, 50, 200, 3, true);
    // img = tmp;
    tmp.copyTo(img);

    // show("canny", 0.25);
}

/**
 * @brief sobelフィルタを適用してedge検出, 結果はEdgeImg::imgに格納
 *
 */
void EdgeImg::detect_edge_with_sobel(const cv::Mat &origin_img)
{
    cv::Mat output, sobel_x, sobel_y, sobel_tmp, tmp;

    // Cannyを使うとなぜだめなのかはメモ参照

    // チャンネル数が1出ない場合は、グレースケールに変換
    if (origin_img.type() != CV_8UC1)
    {
        cv::cvtColor(origin_img, sobel_tmp, cv::COLOR_BGR2GRAY);
    }
    else
    {
        // cv::cvtColor(origin_img, sobel_tmp, cv::COLOR_BGR2GRAY);
        origin_img.copyTo(sobel_tmp);
    }

    cv::GaussianBlur(sobel_tmp, sobel_tmp, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    // NOTE: これ別々にやる必要があるかは 後で調べる
    cv::Sobel(sobel_tmp, sobel_x, CV_64F, 1, 0, 3);
    // cv::Sobel(sobel_tmp, sobel_y, CV_64F, 0, 1, 3);

    // sobel 関数は　karnel3のとき cv::Scharr関数を使ってるらしい
    // cv::Scharr(sobel_tmp, sobel_x, CV_8SC1, 1, 0);
    // cv::Scharr(sobel_tmp, sobel_y, CV_8SC1, 1, 0);

    // cv::addWeighted(sobel_x, 1.0, sobel_y, 1.0, 0.0, tmp);
    sobel_x.copyTo(tmp);
    // show("sobel", 0.25);

    // 絶対値処理をしない場合は、負の値が出てくるので、showは右側のエッジが出てこない。
    cv::convertScaleAbs(tmp, tmp);

    tmp.copyTo(img);
    // show("sobel", 0.25);

    // tmp.copyTo(img);
    // img = tmp;

    // std::cout << "sobel" << img.size() << std::endl;
    // std::cout << "soble" << img.channels() << std::endl;
    // show("sobel:" + name, 0.25);

    // cv::imwrite("sobel:" + name + ".png", tmp);
}

/**
 * @brief 単位球に投影する
 * 点群を極座標に変換し、 r=1の3次元座標に投影する
 *
 * @param projected_img
 */
void InstaImg::convert_to_unitsphere(PointSet &projected_img)
{
    /**
     * @brief uv座標をr=1の極座標に変換に変換する
     *
     */
    auto equirectangular_to_sphere = [=, this](double u, double v)
    {
        u /= width;
        v /= height;

        double phi = u * 2 * M_PI;
        double theta = v * M_PI;

        // NOTE: ここ thetaにabsかけてもいいんですか？
        Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

        return p;
    };

    std::cout << "convert_to_unitsphere" << std::endl;

    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            uchar *ptr = img.data + img.step * v;
            if (ptr[u] != 0)
            {
                // NOTE: ここv, uの順番 逆じゃない？
                // projected_img.add_point(equirectangular_to_sphere(v, u));
                projected_img.add_point(equirectangular_to_sphere(u, v));
            }
        }
    }
}

/**
 * @brief 2つの二値化画像をalpha_blendingする(重ね合わせる). 一つはgreen, もう一つはwhite
 *
 * @param blend_a
 * @param blend_b
 * @param weight 重み(0~1だが、 1.0でOK)
 */
void InstaImg::img_alpha_blending(const cv::Mat &blend_a, const cv::Mat &blend_b, double weight)
{

    cv::Mat out_a = cv::Mat::zeros(blend_a.rows, blend_a.cols, CV_8UC3);
    cv::Mat out_b = cv::Mat::zeros(blend_b.rows, blend_b.cols, CV_8UC3);

    // もしグレースケールであれば 変更する
    (blend_a.channels() == 1) ? cv::cvtColor(blend_a, out_a, cv::COLOR_GRAY2BGR) : blend_a.copyTo(out_a);
    (blend_b.channels() == 1) ? cv::cvtColor(blend_b, out_b, cv::COLOR_GRAY2BGR) : blend_b.copyTo(out_b);

    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            uchar *ptr = img.data + img.step * v;
            if (ptr[u] != 0)
            {
                // blend_a greeen
                out_a.at<cv::Vec3b>(v, u)[0] = 20;
                out_a.at<cv::Vec3b>(v, u)[1] = 255;
                out_a.at<cv::Vec3b>(v, u)[2] = 181;
            }
        }
    }
    // alpha blending
    cv::Mat output = cv::Mat::zeros(blend_a.rows, blend_a.cols, CV_8UC3);
    // cv::addWeighted(out_a, weight, out_b, 1.0 - weight, 0, output);
    cv::addWeighted(out_a, weight, out_b, weight, 0, output);

    // show
    // show("alpha blending:" + name, 0.25, output);
    output.copyTo(img);
    // img = output;
    // cv::imwrite("output_image.png", output);
}

/**
 * @brief closing処理(膨張->収縮)
 *
 * @param count 処理を行う回数
 * @param elem : 0: 長方形 1: 十字型 2: 楕円ぽい感じの
 * @param size カーネルサイズ
 */
void InstaImg::closing(int count, int elem, int size)
{
    for (int i = 0; i < count; i++)
    {
        dilation(elem, size);
    }
    for (int i = 0; i < count; i++)
    {

        erosion(elem, size);
    }
}

/**
 * @brief opening処理(収縮->膨張)
 *
 * @param count 処理を行う回数
 * @param elem : 0: 長方形 1: 十字型 2: 楕円ぽい感じの
 * @param size カーネルサイズ
 */
void InstaImg::opening(int count, int elem, int size)
{
    for (int i = 0; i < count; i++)
    {
        erosion(elem, size);
    }
    for (int i = 0; i < count; i++)
    {

        dilation(elem, size);
    }
}

/**
 * @brief MSEを計算する
 *
 * @param reference 真の画像
 * @param comparison 真の画像にどれだけ近いかを計算する画像
 * @return double MSEの値
 */
double InstaImg::compute_MSE(const cv::Mat &reference, const cv::Mat &comparison)
{

    if (reference.size() != comparison.size())
    {
        std::cout << "size not same" << std::endl;
        std::exit(-1);
    }
    if (reference.channels() != comparison.channels())
    {
        std::cout << "channel not same" << std::endl
                  << "reference: " << reference.channels() << std::endl
                  << "comparison: " << comparison.channels() << std::endl;

        std::exit(-1);
    }

    cv::Scalar_<double> dest = cv::quality::QualityMSE::compute(reference, comparison, cv::noArray());
    // cv::Scalar_<double> dest_ssim = cv::quality::QualitySSIM::compute(reference, comparison, cv::noArray());

    return dest[0];
}

double ImgCalc::compute_MSE(const cv::Mat &reference, const cv::Mat &comparison)
{
    if (reference.size() != comparison.size())
    {
        std::cout << "size not same" << std::endl;
        std::exit(-1);
    }
    if (reference.channels() != comparison.channels())
    {
        std::cout << "channel not same" << std::endl
                  << "reference: " << reference.channels() << std::endl
                  << "comparison: " << comparison.channels() << std::endl;

        std::exit(-1);
    }

    cv::Scalar_<double> dest = cv::quality::QualityMSE::compute(reference, comparison, cv::noArray());

    return dest[0];
}

double ImgCalc::compute_MSE(const std::vector<int> &reference, const std::vector<int> &comparison)
{
    double sum = 0.0;

    // すべての画素値の差の2乗和を計算
    for (int i = 0; i < reference.size(); i++)
    {
        sum += std::pow(reference.at(i) - comparison.at(i), 2);
    }

    return sum / (reference.size() * reference.size());
}

/**
 * @brief 画像を平行移動する はみ出した場所は反対側に移動
 *
 * @param x
 * @param y
 * @param out_mat
 */
std::vector<int> ImgCalc::shift(std::vector<int> target, int size_phi, int size_theta, int x, int y)
{
    std::vector<int> out_mat;
    out_mat.resize(target.size());

    for (int u = 0; u < size_phi; u++)
    {
        for (int v = 0; v < size_theta; v++)
        {
            int ptr = target.at(u * size_theta + v);

            int new_u = (u + y + size_phi) % size_phi;
            int new_v = (v + x + size_theta) % size_theta;
            // std::cout << u << " " << v << " TO ";
            // std::cout << new_u << " " << new_v << std::endl;
            out_mat.at(new_u * size_theta + new_v) = ptr;
        }
    }

    return out_mat;
}


void InstaImg::make_img_from_pointcloud(PointSet &target_point, std::pair<int, int> img_size)
{
    // polarがあるかどうかの確認
    if(target_point.is_empry_polar())
    {
        std::cout << "point_to_img: polar is empty" << std::endl;
        std::exit(-1);
    }


    // 画像の初期化
    cv::Mat imgmat(img_size.first, img_size.second, CV_8UC1, cv::Scalar(0));

    // 変換処理
    for(auto point : target_point.get_point_all_polar())
    {
        // 画像に変換する( theta, phi) -> (v, u)
        double phi = point(1);
        double theta = point(2);
        int u = static_cast<int>(phi / (2.0 * M_PI) * 360.0);
        int v = static_cast<int>(theta / M_PI * 180.0);

        #ifdef _DEBUG
        std::cout << point.transpose() << " ";
        std::cout << u << ":" << v << " " << std::endl;
        #endif

        imgmat.at<uchar>(v, u) = 255;
    }

    set_mat(imgmat);

}





/**
 * @brief plyから得られる点群を全方位画像に投影する
 *
 * @param ply_point 変換する画像
 */
void LidarImg::ply_to_360paranoma_img(PointSet &ply_point)
{
    // 極座標から画像へ投影
    for (auto &point : ply_point.get_point_all_polar())
    {
        double u_dash = point(2) / (2.0 * M_PI);
        double v_dash = point(1) / M_PI;

        // で、おそらく画像は視点座標系で左手系になるので、
        // 右手系と合わせるために、 反転して、90度回転させる
        int u = static_cast<int>(-(u_dash * width) + (width / 4));
        int v = static_cast<int>(v_dash * height);

        if (u > width)
        {
            u -= static_cast<int>(width);
        }

        set_pixel_255(u, v);
    }
}

void LidarImg::ply_to_360paranoma_img_depth(PointSet &ply_point, int depth_flag)
{
    std::cout << "ply_to_360_paranoma_img_depth" << std::endl
              << ply_point.get_point_all().size() << std::endl
              << "polar.num" << ply_point.get_point_all_polar().size() << std::endl;
    assert(ply_point.get_point_all_polar().size() == store_info.size());

    store_pixel.resize(width * height);

    for (const auto &point : ply_point.get_point_all_polar())
    {
        double u_dash = point(2) / (2.0 * M_PI);
        double v_dash = point(1) / M_PI;

        // 画像は左手系なため、右手と合わせるために 反転して90°回転させる
        int u = static_cast<int>(-(u_dash * width) + (width / 4));
        int v = static_cast<int>(v_dash * height);

        if (u > width)
        {
            u -= static_cast<int>(width);
        }
        // 画像の範囲外に出てしまった場合を戻す処理。
        // 下のset_pixel_255は、opencv側でよしなにしてくれるため、ない。
        int new_u = (u + width) % width;
        int new_v = (v + height) % height;
        int pixel = new_u + new_v * width;

        set_store_pixel(pixel, point);
        set_pixel_255(u, v);
    }
    std::cout << "set_store_info_size:" << store_pixel.size() << std::endl;
}

/**
 * @brief plyから得られる点群を全方位画像に投影する。
 *
 * @param ply_point 変換する画像
 * @param flag 画像に点を投影するかどうか
 */
void LidarImg::ply_to_360paranoma_img(PointSet &ply_point, int flag)
{
    std::cout << "ply_to_360paranoma_img" << std::endl
              << "check_point0: " << std::endl
              << ply_point.get_point(0) << std::endl
              << ply_point.get_point_polar(0) << std::endl;

    assert(ply_point.get_point_all_polar().size() == store_info.size());

    size_t index = 0;

    for (const auto &point : ply_point.get_point_all_polar())
    {
        double u_dash = point(2) / (2.0 * M_PI);
        double v_dash = point(1) / M_PI;

        // たぶんだが、画像は左手系なため、右手と合わせるために 反転して90°回転させる
        int u = static_cast<int>(-(u_dash * width) + (width / 4));
        int v = static_cast<int>(v_dash * height);

        if (u > width)
        {
            u -= static_cast<int>(width);
        }

        if (flag == true)
        {
            // 画像の範囲外に出てしまった場合を戻す処理。
            // 下のset_pixel_255は、opencv側でよしなにしてくれるため、ない。
            int new_u = (u + width) % width;
            int new_v = (v + height) % height;
            int pixel = new_u + new_v * width;
            // std::cout << index << " : " << new_u << " " << new_v << " " << pixel << " " << std::endl;

            set_store_info(pixel, index);
        }
        set_pixel_255(u, v);

        index++;
    }

    std::cout << "set_store_info" << store_info.size() << std::endl;
    // 極座標から画像へ投影
    // for (auto &point : ply_point.get_point_all_polar())
    // {
    //     double u_dash = point(2) / (2.0 * M_PI);
    //     double v_dash = point(1) / M_PI;
    //     // で、おそらく画像は視点座標系で左手系になるので、
    //     // 右手系と合わせるために、 反転して、90度回転させる
    //     int u = static_cast<int>(-(u_dash * width) + (width / 4));
    //     int v = static_cast<int>(v_dash * height);

    //     int index = &point - &ply_point.get_point_all_polar()[0];

    //     if (u > width)
    //     {
    //         u -= static_cast<int>(width);
    //     }

    //     if (flag == true)
    //     {
    //         int new_u = (u + img.rows) % img.rows;
    //         int new_v = (v + img.cols) % img.cols;
    //         assert(new_u <= width);
    //         assert(new_v <= height);
    //         assert(index > ply_point.get_point_all().size());
    //         std::cout << "index: " << index << std::endl;
    //         // set_store_info(new_u, new_v, ply_point.get_point(index));
    //     }
    //     set_pixel_255(u, v);
    // }

    // auto polar_iter = ply_point.get_point_all_polar().begin();
    // std::cout << *polar_iter << std::endl;

    // auto polar_end = ply_point.get_point_all_polar().end();
    // auto point_iter = ply_point.get_point_all().begin();

    // while (polar_iter != polar_end)
    // {
    //     auto &point = *polar_iter;
    //     std::cout << point << std::endl;
    //     // auto &nonpolar = *point_iter;

    //     double u_dash = point(2) / (2.0 * M_PI);
    //     double v_dash = point(1) / M_PI;
    //     // で、おそらく画像は視点座標系で左手系になるので、
    //     // 右手系と合わせるために、 反転して、90度回転させる
    //     int u = static_cast<int>(-(u_dash * width) + (width / 4));
    //     int v = static_cast<int>(v_dash * height);
    //     if (u > width)
    //     {
    //         u -= static_cast<int>(width);
    //     }
    //     if (flag == true)
    //     {
    //         int new_u = (u + img.rows) % img.rows;
    //         int new_v = (v + img.cols) % img.cols;
    //         assert(new_u <= width);
    //         assert(new_v <= height);
    //         set_store_info(new_u, new_v, point);
    //     }

    //     set_pixel_255(u, v);
    // ++polar_iter;
    //     ++point_iter;
    // }

    // exit(0);
}

/**
 * @brief 画像を平行移動する はみ出した場所は反対側に移動
 *
 * @param x
 * @param y
 * @param out_mat
 */
cv::Mat InstaImg::shift(int x, int y)
{
    cv::Mat out_mat;
    out_mat = cv::Mat::zeros(img.rows, img.cols, img.type());

    // std::cout << img.rows << " " << img.cols << std::endl;

    for (int v = 0; v < img.cols; v++)
    {
        for (int u = 0; u < img.rows; u++)
        {
            uchar *ptr = img.data + img.step * u;
            if (ptr[v] != 0)
            {
                int new_u = (u + y + img.rows) % img.rows;
                int new_v = (v + x + img.cols) % img.cols;
                out_mat.at<uchar>(new_u, new_v) = ptr[v];
            }
        }
    }

    return out_mat;

    // for (int v = 0; v < img.rows; v++)
    // {
    //     for (int u = 0; u < img.cols; u++)
    //     {
    //         uchar *ptr = img.data + img.step * v;
    //         if (ptr[u] != 0)
    //         {
    //             int new_u = (u + x + img.cols) % img.cols; // はみ出した場合は反対側に移動
    //             int new_v = (v + y + img.rows) % img.rows;

    //             out_mat.at<uchar>(new_v, new_u) = ptr[u];
    //         }
    //     }
    // }
    // return out_mat;
}

void InstaImg::check_pixel_value()
{
    std::cout << "pixel value:" << std::endl;

    for (int i = 1000; i < img.rows / 50; ++i)
    {
        for (int j = 500; j < img.cols / 50; ++j)
        {
            // 画素の値を取得
            // cv::Vec3b pixel = img.at<cv::Vec3b>(i, j);

            // // 各チャンネルの値を表示
            // std::cout << "(" << static_cast<int>(pixel[0]) << ", "
            //           << static_cast<int>(pixel[1]) << ", "
            //           << static_cast<int>(pixel[2]) << ") ";

            // もしグレースケール画像なら channel は 1
            std::cout << img.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief 輝度値の高い点をedgeの可能性が高い点として、LiDARedge画像のエッジの画素の座標値のなかから
 * 高い点をソートして持ってくる関数。
 *
 * あとあとの話、
 * 最小二乗法はすべての点を使った方が精度が上がるため、別関数を作ります。
 * 一応残しておく。
 *
 * @param _img
 */
void InstaImg::diff_pixel(const cv::Mat &_img)
{
    std::cout << "diff_pixel" << std::endl;
    cv::Mat diff_img(img.rows, img.cols, CV_8UC1, cv::Scalar(0));

    struct Pixel_info
    {
        int i;
        int j;
        int pixel_value;
    };

    std::vector<Pixel_info> list_255;

    for (int j = 0; j < img.cols; ++j)
    {
        for (int i = 0; i < img.rows; ++i)
        {
            // それぞれの画素の値を取得
            uchar *ptr = img.data + img.step * i + img.channels() * j;
            uchar *_ptr = _img.data + _img.step * i + _img.channels() * j;

            if (ptr[0] > 200)
            {
                // std::cout << "pixel: " << static_cast<double>(ptr[0]) << " _pixel: " << static_cast<double>(_ptr[0]) << std::endl;

                // diff_img.at<u_char>(i, j) = 255;
                Pixel_info tmp;
                tmp.i = i;
                tmp.j = j;
                tmp.pixel_value = _ptr[0];
                list_255.push_back(tmp);
            }

            // もしグレースケール画像なら channel は 1
            // diff_img.at<double>(i, j) = img.at<double>(i, j) - _img.at<double>(i, j);
        }
    }

    // 高い順にソート
    // 比較関数を定義
    // bool comparePixelValue(const PixelInfo& a, const PixelInfo& b) {
    //     return a.pixelValue < b.pixelValue;
    // }
    // 比較関数はラムダ式で書いているが、定義することも可能
    std::sort(list_255.begin(), list_255.end(), [](Pixel_info a, Pixel_info b)
              { return a.pixel_value > b.pixel_value; });

    // ソート後のベクトルの内容を表示
    int count = 0;

    for (const auto &pixel : list_255)
    {
        std::cout << "{i: " << pixel.i << ", j: " << pixel.j << ", pixelValue: " << pixel.pixel_value << "} ";
        if (count < 30)
        {
            diff_img.at<u_char>(pixel.i, pixel.j) = 255;
            count++;
        }
        else
        {
            break;
        }
    }

    cv::imwrite("diff.png", diff_img);
}

void InstaImg::diff_img(const cv::Mat &_img)
{
    std::cout << "diff_pixel" << std::endl;
    cv::Mat diff_img;
    cv::absdiff(img, _img, diff_img);

    cv::imwrite("diff.png", diff_img);
}

// void LidarImg::set_store_info(int x, int y, Eigen::Vector3d point_data)
// {
//     if (x < 0 || x >= static_cast<int>(store_info.size()) || y < 0 || y >= static_cast<int>(store_info[0].size()))
//     {
//         std::cout << "x or y is over store_info size: " << x << " " << y << std::endl;
//         std::exit(-1);
//     }
//     else
//     {
//         store_info[x][y].push_back(point_data);
//     }
// }

void LidarImg::get_corresponding_point(std::vector<Eigen::Vector3d> &corresp_point, std::vector<std::pair<int, int>> &corresp_pixel, EdgeImg &edge_img_lidar, EdgeImg &edge_img_insta, PointSet &plypoint, int shift_count)
{
    std::cout << "get_corresponding_point" << std::endl;
    cv::Mat insta_edge = edge_img_insta.get_mat();
    cv::Mat lidar_edge = edge_img_lidar.get_mat();

    assert(store_info.size() == plypoint.get_point_all_polar().size());

    InstaImg check;
    check.set_zero_imgMat(img.rows, img.cols, CV_8UC1);

    std::vector<int> corresp_pixel_candidate;

    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            // ここのptrは ply_to_panranomaした直後の画像の値
            uchar *ptr = lidar_edge.data + lidar_edge.step * v;

            uchar *ptr_insta = insta_edge.data + insta_edge.step * v;

            if (ptr[u] > 0 && ptr_insta[u] > 0)
            {
                // 投影したときはshift前なので、変換しておく
                int new_u = (u - shift_count + img.cols) % img.cols;
                int new_v = (v + img.rows) % img.rows;
                corresp_pixel_candidate.push_back(new_v * width + new_u);
                check.set_pixel_255(new_u, new_v);
            }
        }
    }

    check.show("エッジ画像同士の同じ点の出力", 0.25);

    // for (auto &point : store_info)
    // {
    //     std::cout << point << std::endl;
    // }

    // for (auto &point : corresp_pixel_candidatewidth )
    // {
    //     std::cout << point << std::endl;
    // }

    std::cout << "中間結果" << std::endl
              << store_info.size() << std::endl
              << corresp_pixel_candidate.size() << std::endl;

    std::string userInput2;
    std::getline(std::cin, userInput2);

    assert(corresp_pixel_candidate.size() > 0);

    auto compare_distance = [](Eigen::Vector3d a, Eigen::Vector3d b)
    {
        std::cout << a.norm() << " " << b.norm() << std::endl;
        return a.norm() < b.norm();
    };

    // corresp_pixel リストの中から、該当するものをstore_infoから探す
    for (auto &pixel : corresp_pixel_candidate)
    {

        std::vector<Eigen::Vector3d> corresp_point_candidate;
        std::cout << "searching: " << pixel << std::endl;

        size_t index = 0;
        for (auto &projection : store_info)
        {
            if (projection == pixel)
            {
                corresp_point_candidate.push_back(plypoint.get_point(index));
            }
            index++;
        }

        if (corresp_point_candidate.size() > 0)
        {
            int u = pixel % width;
            int v = pixel / width;

            // shiftしたあとの座標に戻す
            u = (u + shift_count + width) % width;

            if (u < 0 || v < 0 || u > width || v > height)
            {
                std::cout << "u or v is over img size: " << u << " " << v << std::endl;
            }

            // std::cout << "found" << u << " " << v << std::endl;
            corresp_pixel.push_back(std::make_pair(u, v));
            corresp_point.push_back(*std::min_element(corresp_point_candidate.begin(), corresp_point_candidate.end(), compare_distance));
        }
        else
        {
            // std::cout << "no corresponding point" << std::endl;
            continue;
        }
    }
}

std::vector<cv::Vec4i> InstaImg::HoughLine_vertical(int threshold, double minline, double maxgap)
{
    cv::Mat dst;
    cv::Canny(img, dst, 50, 200, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, CV_PI / 180, threshold, minline, maxgap);

    return lines;
}

// set_store_infoを使った場合
void LidarImg::get_corresponding_point_Hough_old(std::vector<Eigen::Vector3d> &corresp_point, std::vector<std::pair<int, int>> &corresp_pixel, EdgeImg &edge_img_lidar, EdgeImg &edge_img_insta, PointSet &plypoint, int shift_count, std::string date)
{
    std::cout << "get_corresponding_point_Hough" << std::endl;
    cv::Mat insta_edge = edge_img_insta.get_mat();
    cv::Mat lidar_edge = edge_img_lidar.get_mat();

    assert(store_info.size() == plypoint.get_point_all_polar().size());

    InstaImg candidate_img, candidate_img2, check_candidate;
    candidate_img.set_zero_imgMat(img.rows, img.cols, CV_8UC1);
    candidate_img2.set_zero_imgMat(img.rows, img.cols, CV_8UC1);
    check_candidate.set_zero_imgMat(img.rows, img.cols, CV_8UC1);

    // lineごとに対応点を管理する
    std::vector<std::vector<int>> corresp_pixel_candidate;
    // std::vector<int> corresp_pixel_candidate;

    // 同じ点の座標を求める。
    std::cout << "calc candidate pixel" << std::endl;
    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            // ここのptrは ply_to_panranomaした直後の画像の値
            uchar *ptr = lidar_edge.data + lidar_edge.step * v;

            uchar *ptr_insta = insta_edge.data + insta_edge.step * v;

            if (ptr[u] > 0 && ptr_insta[u] > 0)
            {
                // 投影したときはshift前なので、変換しておく
                int new_u = (u - shift_count + img.cols) % img.cols;
                int new_v = (v + img.rows) % img.rows;
                candidate_img.set_pixel_255(new_u, new_v);
                // corresp_pixel_candidate.push_back(new_v * width + new_u);
            }
        }
    }

    // エッジ画像同士の同じ点の画像から、Hough変換を行う
    std::cout << "calc Hough" << std::endl;

    int _threshold = 50;
    double _minline = 300;
    double _maxgap = 10;

    std::cout << "setting" << _threshold << " " << _minline << " " << _maxgap << std::endl;
    std::vector<cv::Vec4i> lines = candidate_img.HoughLine_vertical(_threshold, _minline, _maxgap);
    // cv::cvtColor(candidate_img.get_mat(), check_candidate.get_mat(), cv::COLOR_GRAY2BGR);
    //
    std::cout << "lines: " << lines.size() << std::endl;

    std::cout << "make_candidate_img2" << std::endl;

    // かぶっている可能性があるので， 外す処理を書いてみる
    std::vector<int> line_overlap_check;

    size_t line_index = 0;
    // Hough変換結果から 縦エッジのみをフィルタリングして、candidate_img2に描画(確認用にcheck_candidateにも描画)
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        int x1 = l[0];
        int y1 = l[1];
        int x2 = l[2];
        int y2 = l[2];

        bool is_overlap = false;

        for (auto &x : line_overlap_check)
        {
            if (x1 == x)
            {
                std::cout << "overlap" << std::endl;
                is_overlap = true;
            }
        }

        if (std::abs(x1 - x2) < 1 && !is_overlap)
        {
            cv::line(check_candidate.get_mat(), cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), 255, 3, cv::LINE_AA);
            cv::line(candidate_img2.get_mat(), cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), 255, 3, cv::LINE_AA);

            // 直線の座標を出力してみる
            // Bresenham's line algorithmというらしい。 PCで直線描画するときに使われてるとか。
            int dx = std::abs(x2 - x1);
            int sx = (x1 < x2) ? 1 : -1;
            int dy = std::abs(y2 - y1);
            int sy = y1 < y2 ? 1 : -1;
            int err = (dx > dy ? dx : -dy) / 2;
            int e2;

            while (true)
            {
                // std::cout << "line: " << i << " " <<  x1 << " " << y1 << std::endl;
                // lineごとの対応を保存
                if (corresp_pixel_candidate.size() < line_index + 1)
                {
                    corresp_pixel_candidate.push_back(std::vector<int>());
                }
                corresp_pixel_candidate.at(line_index).push_back(y1 * width + x1);

                if (x1 == x2 && y1 == y2)
                {
                    break;
                }
                e2 = err;
                if (e2 > -dx)
                {
                    err -= dy;
                    x1 += sx;
                }
                if (e2 < dy)
                {
                    err += dx;
                    y1 += sy;
                }
            }

            line_index++;
        }
        line_overlap_check.push_back(x1);
    }

    // 対応点の座標を求める
    // std::cout << "calc corresp_pixel_candidate" << std::endl;
    // for (int v = 0; v < img.rows; v++)
    // {
    //     for (int u = 0; u < img.cols; u++)
    //     {
    //         uchar *ptr = candidate_img2.get_mat().data + candidate_img2.get_mat().step * v;

    //         if (ptr[u] > 0)
    //         {
    //             corresp_pixel_candidate.push_back(v * width + u);
    //         }
    //     }
    // }

    cv::imwrite("out/" + date + "/" + date + "houghline_vertical_check.png", check_candidate.get_mat());
    cv::imwrite("out/" + date + "/" + date + "houghline_vertical_img.png", candidate_img.get_mat());
    // cv::imwrite("houghline_vertical_img2.png", candidate_img2.get_mat());

    std::cout << "中間結果" << std::endl
              << store_info.size() << std::endl
              << corresp_pixel_candidate.size() << std::endl;

    std::string userInput2;
    std::getline(std::cin, userInput2);

    std::ofstream ofs("corresp_pixel_candidate.txt");

    assert(corresp_pixel_candidate.size() > 0);

    auto compare_distance = [](Eigen::Vector3d a, Eigen::Vector3d b)
    {
        std::cout << "compare_distance" << a.norm() << " " << b.norm() << std::endl;
        return a.norm() < b.norm();
    };

    // pixelから 画像対応点に変換する
    auto pixel_to_uv = [this, shift_count](int pixel)
    {
        int u = pixel % width;
        int v = pixel / width;

        // shiftしたあとの座標に戻す
        u = (u + shift_count + width) % width;

        if (u < 0 || v < 0 || u > width || v > height)
        {
            std::cout << "u or v is over img size: " << u << " " << v << std::endl;
        }
        return std::make_pair(u, v);
    };

    struct candidate_pair
    {
        int pixel;
        Eigen::Vector3d point;
        int store_index;
    };

    int search_line_index = 0;
    // lineごとに該当する点をstore_infoから探す
    for (auto &line_pixel : corresp_pixel_candidate)
    {
        // ヒストグラムの階級数
        // int bin_width = 1 + static_cast<int>(std::log2(line_pixel.size()));
        double bin_width = 0.05;

        std::cout << "search_line" << search_line_index << std::endl;
        std::cout << "bin_width" << bin_width << std::endl;

        // 各ヒストグラムと
        std::vector<double> distance_from_center;
        // 個数のヒストグラムと それに対応するpixel pointのpairをそれぞれ格納
        std::array<int, 500> histgram_distance;
        std::vector<candidate_pair> candidate_corresp;

        // histgram_distanceを0で初期化
        std::fill(histgram_distance.begin(), histgram_distance.end(), 0);
        // candidate_correspを0で初期化
        // std::fill(candidate_corresp.begin(), candidate_corresp.end(), candidate_pair{0, 0});

        // corresp_pixel リストの中から、該当するものをstore_infoから探す
        for (auto &pixel : line_pixel)
        {

            std::vector<Eigen::Vector3d> corresp_point_candidate;
            // std::cout << "searching: " << pixel << std::endl;

            size_t index = 0;
            // store_info(1の点はpixelのどこに投影されたかを記録してるので)
            // 同じpixelがあるか探索(まああるんだけど)
            for (auto &projection : store_info)
            {
                if (projection == pixel)
                {
                    corresp_point_candidate.push_back(plypoint.get_point(index));
                }
                index++;
            }

            // candidate_pixelに対応する点が見つかったら
            if (corresp_point_candidate.size() > 0)
            {
                // corresp_pixel.push_back(pixel_to_uv(pixel));
                // corresp_point.push_back(*std::min_element(corresp_point_candidate.begin(), corresp_point_candidate.end(), compare_distance));
                // 格納する点の距離をbin_widthで割って histgramに格納
                Eigen::Vector3d point = *std::min_element(corresp_point_candidate.begin(), corresp_point_candidate.end(), compare_distance);
                double point_dis = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0));
                int hist_index = point_dis / bin_width;

                std::cout << "hist_index" << hist_index << " distance:" << point_dis << std::endl;
                // ヒストグラムの更新
                histgram_distance.at(hist_index)++;
                candidate_corresp.push_back({pixel, point, hist_index});
            }
            else
            {
                // std::cout << "no corresponding point" << std::endl;
                continue;
            }
        }

        // ヒストグラムの中から大きいものではなく 前面でほしいものを探す
        // auto iter = std::find_if(histgram_distance.begin(), histgram_distance.end(), [](int value)
        //                          { return value != 0; });
        // auto c_index = std::distance(histgram_distance.begin(), iter);

        // ヒストグラムの中から最大の物を探す
        auto iter = std::max_element(histgram_distance.begin(), histgram_distance.end());
        auto c_index = std::distance(histgram_distance.begin(), iter);

        std::cout << "c_index" << c_index << std::endl;

        if (c_index == histgram_distance.size())
        {
            std::cout << "no corresponding point" << std::endl;
            continue;
        }
        else
        {
            // print
            ofs << "next_line" << std::endl;
            for (unsigned int i = 0; i < histgram_distance.size(); i++)
            {
                if (histgram_distance.at(i) != 0)
                {
                    ofs << histgram_distance.at(i) << " " << i * bin_width << std::endl;
                    std::cout << histgram_distance.at(i) << " " << i * bin_width << std::endl;
                }
            }

            for (auto &candidate : candidate_corresp)
            {
                if (candidate.store_index == c_index)
                {
                    std::cout << "found: " << candidate.pixel << " " << candidate.point.transpose();
                    corresp_pixel.push_back(pixel_to_uv(candidate.pixel));
                    corresp_point.push_back(candidate.point);
                }
            }
        }

        search_line_index++;
    }
    ofs.close();
}

void LidarImg::set_store_pixel(int pixel, Eigen::Vector3d new_point)
{
    double new_distance = std::sqrt(std::pow(new_point(0), 2.0) + std::pow(new_point(1), 2.0));
    double distance = store_pixel.at(pixel).distance;

    if (new_distance < distance || distance == 0.0)
    {
        store_pixel.at(pixel).point = new_point;
        store_pixel.at(pixel).distance = new_distance;
    }
}

// store_pixelを使う
void LidarImg::get_corresponding_point_Hough(std::vector<Eigen::Vector3d> &corresp_point, std::vector<std::pair<int, int>> &corresp_pixel, EdgeImg &edge_img_lidar, EdgeImg &edge_img_insta, PointSet &plypoint, int shift_count, std::string date)
{
    std::cout << "get_corresponding_point_Hough" << std::endl;
    cv::Mat insta_edge = edge_img_insta.get_mat();
    cv::Mat lidar_edge = edge_img_lidar.get_mat();

    assert(store_info.size() == plypoint.get_point_all_polar().size());

    InstaImg candidate_img, candidate_img2, check_candidate;
    candidate_img.set_zero_imgMat(img.rows, img.cols, CV_8UC1);
    candidate_img2.set_zero_imgMat(img.rows, img.cols, CV_8UC1);
    check_candidate.set_zero_imgMat(img.rows, img.cols, CV_8UC1);

    // lineごとに対応点を管理する
    std::vector<std::vector<int>> corresp_pixel_candidate;
    // std::vector<int> corresp_pixel_candidate;

    // 同じ点の座標を求める。
    std::cout << "calc candidate pixel" << std::endl;
    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            // ここのptrは ply_to_panranomaした直後の画像の値
            uchar *ptr = lidar_edge.data + lidar_edge.step * v;
            uchar *ptr_insta = insta_edge.data + insta_edge.step * v;

            if (ptr[u] > 0 && ptr_insta[u] > 0)
            {
                // 投影したときはshift前なので、変換しておく
                int new_u = (u - shift_count + img.cols) % img.cols;
                int new_v = (v + img.rows) % img.rows;
                candidate_img.set_pixel_255(new_u, new_v);
            }
        }
    }

    // エッジ画像同士の同じ点の画像から、Hough変換を行う
    std::cout << "calc Hough" << std::endl;

    int _threshold = 50;
    double _minline = 100;
    double _maxgap = 10;

    std::cout << "setting" << _threshold << " " << _minline << " " << _maxgap << std::endl;
    std::vector<cv::Vec4i> lines = candidate_img.HoughLine_vertical(_threshold, _minline, _maxgap);
    std::cout << "lines: " << lines.size() << std::endl;
    std::cout << "make_candidate_img2" << std::endl;

    // かぶっている可能性があるので， 外す処理を書いてみる
    std::vector<int> line_overlap_check;

    size_t line_index = 0;
    // Hough変換結果から 縦エッジのみをフィルタリングして、candidate_img2に描画(確認用にcheck_candidateにも描画)
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        int x1 = l[0];
        int y1 = l[1];
        int x2 = l[2];
        int y2 = l[2];

        bool is_overlap = false;

        // 直線がかぶっていたら 除く
        for (auto &x : line_overlap_check)
        {
            if (x1 == x)
            {
                std::cout << "overlap" << std::endl;
                is_overlap = true;
            }
        }

        if (std::abs(x1 - x2) < 1 && !is_overlap)
        {
            cv::line(check_candidate.get_mat(), cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), 255, 3, cv::LINE_AA);
            cv::line(candidate_img2.get_mat(), cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), 255, 3, cv::LINE_AA);

            // 直線の座標を出力してみる
            // Bresenham's line algorithmというらしい。 PCで直線描画するときに使われてるとか。
            int dx = std::abs(x2 - x1);
            int sx = (x1 < x2) ? 1 : -1;
            int dy = std::abs(y2 - y1);
            int sy = y1 < y2 ? 1 : -1;
            int err = (dx > dy ? dx : -dy) / 2;
            int e2;

            while (true)
            {
                // std::cout << "line: " << i << " " <<  x1 << " " << y1 << std::endl;
                // lineごとの対応を保存
                if (corresp_pixel_candidate.size() < line_index + 1)
                {
                    corresp_pixel_candidate.push_back(std::vector<int>());
                }
                corresp_pixel_candidate.at(line_index).push_back(y1 * width + x1);

                if (x1 == x2 && y1 == y2)
                {
                    break;
                }
                e2 = err;
                if (e2 > -dx)
                {
                    err -= dy;
                    x1 += sx;
                }
                if (e2 < dy)
                {
                    err += dx;
                    y1 += sy;
                }
            }

            line_index++;
        }
        line_overlap_check.push_back(x1);
    }

    cv::imwrite("out/" + date + "/" + date + "houghline_vertical_check.png", check_candidate.get_mat());
    cv::imwrite("out/" + date + "/" + date + "houghline_vertical_img.png", candidate_img.get_mat());
    // cv::imwrite("houghline_vertical_img2.png", candidate_img2.get_mat());

    std::cout << "中間結果" << std::endl
              << store_info.size() << std::endl
              << corresp_pixel_candidate.size() << std::endl;

    std::string userInput2;
    std::getline(std::cin, userInput2);

    std::ofstream ofs("corresp_pixel_candidate.txt");
    assert(corresp_pixel_candidate.size() > 0);

    // pixelから 画像対応点に変換する
    auto pixel_to_uv = [this, shift_count](int pixel)
    {
        int u = pixel % width;
        int v = pixel / width;

        // shiftしたあとの座標に戻す
        u = (u + shift_count + width) % width;

        if (u < 0 || v < 0 || u > width || v > height)
        {
            std::cout << "u or v is over img size: " << u << " " << v << std::endl;
        }
        return std::make_pair(u, v);
    };

    struct candidate_pair
    {
        int pixel;
        Eigen::Vector3d point;
        int store_index;
    };

    int search_line_index = 0;
    // lineごとに該当する点をstore_pixelから探す
    for (auto &line_pixel : corresp_pixel_candidate)
    {
        // ヒストグラムの階級数
        // int bin_width = 1 + static_cast<int>(std::log2(line_pixel.size()));
        double bin_width = 0.05;

        std::cout << "search_line" << search_line_index << std::endl;
        std::cout << "bin_width" << bin_width << std::endl;

        // 各ヒストグラムと
        std::vector<double> distance_from_center;
        // 個数のヒストグラムと それに対応するpixel pointのpairをそれぞれ格納
        std::array<int, 500> histgram_distance;
        std::vector<candidate_pair> candidate_corresp;

        // histgram_distanceを0で初期化
        std::fill(histgram_distance.begin(), histgram_distance.end(), 0);
        // candidate_correspを0で初期化
        // std::fill(candidate_corresp.begin(), candidate_corresp.end(), candidate_pair{0, 0});

        // lineごとの対応のあるpixelを探索
        for (auto &pixel : line_pixel)
        {
            if (pixel > height * width)
            {
                std::cout << pixel << "pixel is over" << height * width << std::endl;
                std::cout << store_pixel.capacity() << std::endl;
                continue;
            }
            auto store_pixell = store_pixel.at(pixel);
            // pixelに対応するstore_pixelの点のうち 距離が0でないものを探す
            if (store_pixell.distance != 0)
            {
                double point_dis = store_pixell.distance;
                int hist_index = point_dis / bin_width;
                histgram_distance.at(hist_index)++;
                candidate_corresp.push_back({pixel, store_pixell.point, hist_index});
            }
            else
            {
                // std::cout << "no corresponding point" << std::endl;
                continue;
            }
        }

        // ヒストグラムの中から大きいものではなく 前面でほしいものを探す
        // auto iter = std::find_if(histgram_distance.begin(), histgram_distance.end(), [](int value)
        //                          { return value != 0; });
        // auto c_index = std::distance(histgram_distance.begin(), iter);

        // ヒストグラムの中から最大の物を探す
        auto iter = std::max_element(histgram_distance.begin(), histgram_distance.end());
        auto c_index = std::distance(histgram_distance.begin(), iter);

        std::cout << "c_index" << c_index << std::endl;

        if (c_index == histgram_distance.size())
        {
            std::cout << "no corresponding point" << std::endl;
            continue;
        }
        else
        {
            // print
            ofs << "next_line" << std::endl;
            for (unsigned int i = 0; i < histgram_distance.size(); i++)
            {
                if (histgram_distance.at(i) != 0)
                {
                    ofs << histgram_distance.at(i) << " " << i * bin_width << std::endl;
                    std::cout << histgram_distance.at(i) << " " << i * bin_width << std::endl;
                }
            }

            for (auto &candidate : candidate_corresp)
            {
                if (candidate.store_index == c_index)
                {
                    std::cout << "found: " << candidate.pixel << " " << candidate.point.transpose();
                    corresp_pixel.push_back(pixel_to_uv(candidate.pixel));
                    corresp_point.push_back(candidate.point);
                }
            }
        }

        search_line_index++;
    }
    ofs.close();
}
