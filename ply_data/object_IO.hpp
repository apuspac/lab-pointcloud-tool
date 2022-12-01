#ifndef OBJECT_IO_HPP_INCLUDE_GUARD
#define OBJECT_IO_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "operation.hpp"
#include <fstream>
#include <unistd.h>
#include <getopt.h>

/**
 * @brief imgfile pointfile plyfileの読み込み
 *
 */
class ObjectIO
{
public:
    void print();
    static void option_process(int, char **, PointOperation);
    static void load_ply_file(std::string, std::string, int, PointSet *);
};

#endif