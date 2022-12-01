#include "operation.hpp"

int PointOperation::option_process(int argc, char **argv)
{

    int opt;
    for (int i = 0; i < argc; i++)
    {
        std::cout << argv[i] << std::endl;
    }

    // m: imgの対応点 .dat
    // x: plyの対応点 .dat
    // p: plyファイル .ply
    // i: imgファイル .jpg,png
    // d: デフォルトdirpath
    const char opt_string = "m:x:p:d:i:";

    // static struct option long_options[] =
    //     {
    //         {"",required_argument,NULL,

    //         }
    //     };

    opterr = 0;
    while ((opt = getopt(argc, argv, opt_string)) != -1)
    {
        switch (opt)
        {
        case "m":
            tmp = (optarg);
            std::cout << tmp << std::endl;
            break;

        case "x":
            auto tmp = (optarg);
            std::cout << tmp << std::endl;
            break;

        case "p":
            auto tmp = (optarg);
            std::cout << tmp << std::endl;
            break;

        case "i":
            auto tmp = (optarg);
            std::cout << tmp << std::endl;
            break;

        case "d":
            auto tmp = (optarg);
            std::cout << tmp << std::endl;
            break;
        default:
            break;
        }
    }

    return 0;
    // while((opt = getopt(argc, argv, "")))

    // // コマンドオプション処理
    // char opt;
    // int i;

    // std::string file_path_1;
    // std::string file_path_2;
    // std::string file_path_3;

    // std::string out_path;
    // std::string img_path;

    // // コマンドライン引数のオプションがなくなるまで繰り返す
    // //  getoputの第3引数にオプションの文字列を指定する。引数撮る場合は":"をつける
    // //  a,cは引数をとらないが、 bは引数をとる。

    // optind = 0;
    // while ((opt = getopt(argc, argv, "i:")) != -1)
    // {
    //     switch (opt)
    //     {
    //     case 'i':

    //         if (optarg[0] == '-')
    //         {
    //             printf("Option [%c] requires two arguments\n", opt);
    //             std::cout << "opt";
    //             return -1;
    //         }

    //         optind--;
    //         file_path_1 = argv[optind++];
    //         file_path_2 = argv[optind++];
    //         file_path_3 = argv[optind++];
    //         img_path = argv[optind++];
    //         out_path = argv[optind];

    //         // マジでオプション処理の動きがわからない
    //         //  for (i = 0; i < 1; i++)
    //         //  {

    //         //     if (argv[optind][0] == '-')
    //         //     {
    //         //         printf("error!");
    //         //         return -1;
    //         //     }
    //         //     file_path_2 = argv[optind++];
    //         // }

    //         printf("\n");
    //         break;
    //     default:
    //         printf("Unknown option '%c'\n", opt);
    //         break;
    //     }
    //     optarg = NULL;
    // }
    // /* オプション以外の引数を表示する。 */
    // if (optind < argc)
    // {
    //     while (optind < argc)
    //     {
    //         optind++;
    //         // printf("Not Option str '%s'\n", argv[optind++]);
    //     }
    // }

    // std::cout << "img_point :" << file_path_1 << std::endl;
    // std::cout << "ply_point :" << file_path_2 << std::endl;
    // std::cout << "plyfile :" << file_path_3 << std::endl
    //           << std::endl;
    // std::cout << "imgpath :" << img_path << std::endl;
    // std::cout << "outdir :" << out_path << std::endl;

    // transform_coordinate(file_path_1, file_path_2, file_path_3, img_path, out_path);
}

void PointSetIO::print()
{
    std::cout << "operation load" << std::endl;
}