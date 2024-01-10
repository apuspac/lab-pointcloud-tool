/**
 * @file img_proc.cpp
 * @brief
 *
 *
 */

#include "img_proc.hpp"

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
    img.at<u_char>(v, u) = 255;
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

/**
 * @brief plyから得られる点群を全方位画像に投影する。
 *
 * @param ply_point 変換する画像
 * @param flag 画像に点を投影するかどうか
 */
void LidarImg::ply_to_360paranoma_img(PointSet &ply_point, int flag)
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

        if (flag == true)
        {
            set_store_info(u, v, point);
        }
        set_pixel_255(u, v);
    }
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
 * @brief 同じ点を取ってくる。 輝度値で判断しようとしたが、最小二乗法はすべて使った方が精度が上がるため、 全部使った方が良いため、別関数を作ります。
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