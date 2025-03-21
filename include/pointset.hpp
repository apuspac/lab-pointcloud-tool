/**
 * @file pointset.hpp
 * @brief 点群・plyファイルを扱う
 */
#ifndef POINTSET_HPP_INCLUDE_GUARD
#define POINTSET_HPP_INCLUDE_GUARD

#include <vector>
#include <float.h>
#include <math.h>
#include <iomanip>
#include <unordered_map>
#include <numeric>
#include <mutex>
#include <fstream>

#include <eigen3/Core>
#include <eigen3/LU>
#include <eigen3/Dense>


#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#endif

/**
 * @brief 点群そのものに関するクラス
 *
 */
class PointSet
{
private:
    // 点群を扱う。 点単体は Eigen::Vector3
    std::vector<Eigen::Vector3d> point3;
    // 極座標(r, theta, phi)
    std::vector<Eigen::Vector3d> point3_polar;
    std::vector<std::array<int, 2>> edge2;
    Eigen::Vector3d center_of_gravity;
    std::string name;

    // parts検出用
    int class_num;
    std::string class_name;

    // histgram
    std::array<int, 1000> histgram_intervals;

public:
    PointSet(std::string point_name = "none") : name(point_name) { histgram_intervals = {}; center_of_gravity = Eigen::Vector3d(0, 0, 0); }
    ~PointSet() {}

    // set
    void set_pointset_name(std::string _name) { name = _name; }
    void set_class_name(std::string _name) { class_name = _name; }
    void set_class_num(int _num) { class_num = _num; }

    // get
    long unsigned int get_point_num() { return point3.size(); }
    long unsigned int get_point_num_polar() { return point3_polar.size(); }
    long unsigned int get_edge_num() { return edge2.size(); }
    int get_class_num() { return class_num; }
    std::string get_class_name() { return class_name; }
    std::string get_name() { return name; }

    // is
    bool is_empty() { return point3.size() == 0; }
    bool is_empty_edge() { return edge2.size() == 0; }
    bool is_empty_polar() { return point3_polar.size() == 0; }

    // 点の追加 edgeの組を追加
    void add_point(const Eigen::Vector3d add_point) { point3.push_back(add_point); }
    void add_point(PointSet);
    void add_point_polar(const Eigen::Vector3d add_point) { point3_polar.push_back(add_point); }
    void add_point_polar(PointSet);
    void add_edge(std::array<int, 2> edge) { edge2.push_back(edge); }
    /**
     * @brief Get the point object
     * 単体が欲しいとき用だが、 get_point_all()で渡して そっちで.at()したほうがいいかもしれない
     *
     * @param i 欲しい点の番号
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d get_point(uint64_t i) { return point3.at(i); }
    Eigen::Vector3d get_point_polar(uint64_t i) { return point3_polar.at(i); }
    std::array<int, 2> get_edge(uint64_t i) { return edge2.at(i); }
    // Eigen::Vector3d get_center_of_gravity() { return center_of_gravity; }
    Eigen::Vector3d get_center_of_gravity();

    // 全体getter
    std::vector<Eigen::Vector3d> get_point_all() { return point3; }
    std::vector<Eigen::Vector3d> get_point_all_polar()
    {
        assert(point3_polar.size() != 0);
        return point3_polar;
    }
    std::vector<std::array<int, 2>> get_edge_all() { return edge2; }

    // remove

    // calc
    void convert_to_polar();
    void convert_to_rectangular();
    void create_histgram();
    void calc_center_of_gravity();
    void rotate(Eigen::Matrix3d);
    void transform(Eigen::Vector3d);

    void output_hist(std::string);
    void print();
    void print_polar();

    void cutting_by_height(double, bool);

#ifdef OPEN3D_ENABLED
    void radius_based_filter(size_t, double);
#endif
};
#endif
