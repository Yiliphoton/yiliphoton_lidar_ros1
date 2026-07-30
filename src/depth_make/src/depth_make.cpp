#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
//  #include<pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <vector>

#ifdef PCL_USE_PCDIO
#include <pcl/io/pcd_io.h>
#else
#include <pcl/io/io.h>
#endif
// #include<pcl_ros/filters/filter.h>
// #include<pcl_ros/point_cloud.h>
// #include<opencv2/core/eigen.hpp>
// #include<opencv2/opencv.hpp>
// #include<cv_bridge/cv_bridge.h>
// #include<opencv2/highgui/highgui.hpp>

#include"../include/depth_make/make_pointxyzitr.h"
//make_pointxyzitr.h为自定义PCL点云消息pcl::PointCloud<pcl::PointXYZITR>的头文件，该自定义点云消息中的每个点都包括了XYZ坐标、反射强度intensity、时间戳time_stamp和点所在线束ring。
 
using namespace std;

ros::Publisher pub;
ros::Publisher pub_rgb;
const size_t pixel_value_max = std::numeric_limits<uint16_t>::max();
float min_depth = std::numeric_limits<float>::max();
float max_depth = std::numeric_limits<float>::min();

const size_t viridis_n = 256;
const float viridis_palette[viridis_n][3] = {
    {0.26700401f, 0.00487433f, 0.32941519f},
    {0.26851048f, 0.00960483f, 0.33542652f},
    {0.26994384f, 0.01462494f, 0.34137895f},
    {0.27130489f, 0.01994186f, 0.34726862f},
    {0.27259384f, 0.02556309f, 0.35309303f},
    {0.27380934f, 0.03149748f, 0.35885256f},
    {0.27495242f, 0.03775181f, 0.36454323f},
    {0.27602238f, 0.04416723f, 0.37016418f},
    {0.2770184f, 0.05034437f, 0.37571452f},
    {0.27794143f, 0.05632444f, 0.38119074f},
    {0.27879067f, 0.06214536f, 0.38659204f},
    {0.2795655f, 0.06783587f, 0.39191723f},
    {0.28026658f, 0.07341724f, 0.39716349f},
    {0.28089358f, 0.07890703f, 0.40232944f},
    {0.28144581f, 0.0843197f, 0.40741404f},
    {0.28192358f, 0.08966622f, 0.41241521f},
    {0.28232739f, 0.09495545f, 0.41733086f},
    {0.28265633f, 0.10019576f, 0.42216032f},
    {0.28291049f, 0.10539345f, 0.42690202f},
    {0.28309095f, 0.11055307f, 0.43155375f},
    {0.28319704f, 0.11567966f, 0.43611482f},
    {0.28322882f, 0.12077701f, 0.44058404f},
    {0.28318684f, 0.12584799f, 0.44496f},
    {0.283072f, 0.13089477f, 0.44924127f},
    {0.28288389f, 0.13592005f, 0.45342734f},
    {0.28262297f, 0.14092556f, 0.45751726f},
    {0.28229037f, 0.14591233f, 0.46150995f},
    {0.28188676f, 0.15088147f, 0.46540474f},
    {0.28141228f, 0.15583425f, 0.46920128f},
    {0.28086773f, 0.16077132f, 0.47289909f},
    {0.28025468f, 0.16569272f, 0.47649762f},
    {0.27957399f, 0.17059884f, 0.47999675f},
    {0.27882618f, 0.1754902f, 0.48339654f},
    {0.27801236f, 0.18036684f, 0.48669702f},
    {0.27713437f, 0.18522836f, 0.48989831f},
    {0.27619376f, 0.19007447f, 0.49300074f},
    {0.27519116f, 0.1949054f, 0.49600488f},
    {0.27412802f, 0.19972086f, 0.49891131f},
    {0.27300596f, 0.20452049f, 0.50172076f},
    {0.27182812f, 0.20930306f, 0.50443413f},
    {0.27059473f, 0.21406899f, 0.50705243f},
    {0.26930756f, 0.21881782f, 0.50957678f},
    {0.26796846f, 0.22354911f, 0.5120084f},
    {0.26657984f, 0.2282621f, 0.5143487f},
    {0.2651445f, 0.23295593f, 0.5165993f},
    {0.2636632f, 0.23763078f, 0.51876163f},
    {0.26213801f, 0.24228619f, 0.52083736f},
    {0.26057103f, 0.2469217f, 0.52282822f},
    {0.25896451f, 0.25153685f, 0.52473609f},
    {0.25732244f, 0.2561304f, 0.52656332f},
    {0.25564519f, 0.26070284f, 0.52831152f},
    {0.25393498f, 0.26525384f, 0.52998273f},
    {0.25219404f, 0.26978306f, 0.53157905f},
    {0.25042462f, 0.27429024f, 0.53310261f},
    {0.24862899f, 0.27877509f, 0.53455561f},
    {0.2468114f, 0.28323662f, 0.53594093f},
    {0.24497208f, 0.28767547f, 0.53726018f},
    {0.24311324f, 0.29209154f, 0.53851561f},
    {0.24123708f, 0.29648471f, 0.53970946f},
    {0.23934575f, 0.30085494f, 0.54084398f},
    {0.23744138f, 0.30520222f, 0.5419214f},
    {0.23552606f, 0.30952657f, 0.54294396f},
    {0.23360277f, 0.31382773f, 0.54391424f},
    {0.2316735f, 0.3181058f, 0.54483444f},
    {0.22973926f, 0.32236127f, 0.54570633f},
    {0.22780192f, 0.32659432f, 0.546532f},
    {0.2258633f, 0.33080515f, 0.54731353f},
    {0.22392515f, 0.334994f, 0.54805291f},
    {0.22198915f, 0.33916114f, 0.54875211f},
    {0.22005691f, 0.34330688f, 0.54941304f},
    {0.21812995f, 0.34743154f, 0.55003755f},
    {0.21620971f, 0.35153548f, 0.55062743f},
    {0.21429757f, 0.35561907f, 0.5511844f},
    {0.21239477f, 0.35968273f, 0.55171011f},
    {0.2105031f, 0.36372671f, 0.55220646f},
    {0.20862342f, 0.36775151f, 0.55267486f},
    {0.20675628f, 0.37175775f, 0.55311653f},
    {0.20490257f, 0.37574589f, 0.55353282f},
    {0.20306309f, 0.37971644f, 0.55392505f},
    {0.20123854f, 0.38366989f, 0.55429441f},
    {0.1994295f, 0.38760678f, 0.55464205f},
    {0.1976365f, 0.39152762f, 0.55496905f},
    {0.19585993f, 0.39543297f, 0.55527637f},
    {0.19410009f, 0.39932336f, 0.55556494f},
    {0.19235719f, 0.40319934f, 0.55583559f},
    {0.19063135f, 0.40706148f, 0.55608907f},
    {0.18892259f, 0.41091033f, 0.55632606f},
    {0.18723083f, 0.41474645f, 0.55654717f},
    {0.18555593f, 0.4185704f, 0.55675292f},
    {0.18389763f, 0.42238275f, 0.55694377f},
    {0.18225561f, 0.42618405f, 0.5571201f},
    {0.18062949f, 0.42997486f, 0.55728221f},
    {0.17901879f, 0.43375572f, 0.55743035f},
    {0.17742298f, 0.4375272f, 0.55756466f},
    {0.17584148f, 0.44128981f, 0.55768526f},
    {0.17427363f, 0.4450441f, 0.55779216f},
    {0.17271876f, 0.4487906f, 0.55788532f},
    {0.17117615f, 0.4525298f, 0.55796464f},
    {0.16964573f, 0.45626209f, 0.55803034f},
    {0.16812641f, 0.45998802f, 0.55808199f},
    {0.1666171f, 0.46370813f, 0.55811913f},
    {0.16511703f, 0.4674229f, 0.55814141f},
    {0.16362543f, 0.47113278f, 0.55814842f},
    {0.16214155f, 0.47483821f, 0.55813967f},
    {0.16066467f, 0.47853961f, 0.55811466f},
    {0.15919413f, 0.4822374f, 0.5580728f},
    {0.15772933f, 0.48593197f, 0.55801347f},
    {0.15626973f, 0.4896237f, 0.557936f},
    {0.15481488f, 0.49331293f, 0.55783967f},
    {0.15336445f, 0.49700003f, 0.55772371f},
    {0.1519182f, 0.50068529f, 0.55758733f},
    {0.15047605f, 0.50436904f, 0.55742968f},
    {0.14903918f, 0.50805136f, 0.5572505f},
    {0.14760731f, 0.51173263f, 0.55704861f},
    {0.14618026f, 0.51541316f, 0.55682271f},
    {0.14475863f, 0.51909319f, 0.55657181f},
    {0.14334327f, 0.52277292f, 0.55629491f},
    {0.14193527f, 0.52645254f, 0.55599097f},
    {0.14053599f, 0.53013219f, 0.55565893f},
    {0.13914708f, 0.53381201f, 0.55529773f},
    {0.13777048f, 0.53749213f, 0.55490625f},
    {0.1364085f, 0.54117264f, 0.55448339f},
    {0.13506561f, 0.54485335f, 0.55402906f},
    {0.13374299f, 0.54853458f, 0.55354108f},
    {0.13244401f, 0.55221637f, 0.55301828f},
    {0.13117249f, 0.55589872f, 0.55245948f},
    {0.1299327f, 0.55958162f, 0.55186354f},
    {0.12872938f, 0.56326503f, 0.55122927f},
    {0.12756771f, 0.56694891f, 0.55055551f},
    {0.12645338f, 0.57063316f, 0.5498411f},
    {0.12539383f, 0.57431754f, 0.54908564f},
    {0.12439474f, 0.57800205f, 0.5482874f},
    {0.12346281f, 0.58168661f, 0.54744498f},
    {0.12260562f, 0.58537105f, 0.54655722f},
    {0.12183122f, 0.58905521f, 0.54562298f},
    {0.12114807f, 0.59273889f, 0.54464114f},
    {0.12056501f, 0.59642187f, 0.54361058f},
    {0.12009154f, 0.60010387f, 0.54253043f},
    {0.11973756f, 0.60378459f, 0.54139999f},
    {0.11951163f, 0.60746388f, 0.54021751f},
    {0.11942341f, 0.61114146f, 0.53898192f},
    {0.11948255f, 0.61481702f, 0.53769219f},
    {0.11969858f, 0.61849025f, 0.53634733f},
    {0.12008079f, 0.62216081f, 0.53494633f},
    {0.12063824f, 0.62582833f, 0.53348834f},
    {0.12137972f, 0.62949242f, 0.53197275f},
    {0.12231244f, 0.63315277f, 0.53039808f},
    {0.12344358f, 0.63680899f, 0.52876343f},
    {0.12477953f, 0.64046069f, 0.52706792f},
    {0.12632581f, 0.64410744f, 0.52531069f},
    {0.12808703f, 0.64774881f, 0.52349092f},
    {0.13006688f, 0.65138436f, 0.52160791f},
    {0.13226797f, 0.65501363f, 0.51966086f},
    {0.13469183f, 0.65863619f, 0.5176488f},
    {0.13733921f, 0.66225157f, 0.51557101f},
    {0.14020991f, 0.66585927f, 0.5134268f},
    {0.14330291f, 0.66945881f, 0.51121549f},
    {0.1466164f, 0.67304968f, 0.50893644f},
    {0.15014782f, 0.67663139f, 0.5065889f},
    {0.15389405f, 0.68020343f, 0.50417217f},
    {0.15785146f, 0.68376525f, 0.50168574f},
    {0.16201598f, 0.68731632f, 0.49912906f},
    {0.1663832f, 0.69085611f, 0.49650163f},
    {0.1709484f, 0.69438405f, 0.49380294f},
    {0.17570671f, 0.6978996f, 0.49103252f},
    {0.18065314f, 0.70140222f, 0.48818938f},
    {0.18578266f, 0.70489133f, 0.48527326f},
    {0.19109018f, 0.70836635f, 0.48228395f},
    {0.19657063f, 0.71182668f, 0.47922108f},
    {0.20221902f, 0.71527175f, 0.47608431f},
    {0.20803045f, 0.71870095f, 0.4728733f},
    {0.21400015f, 0.72211371f, 0.46958774f},
    {0.22012381f, 0.72550945f, 0.46622638f},
    {0.2263969f, 0.72888753f, 0.46278934f},
    {0.23281498f, 0.73224735f, 0.45927675f},
    {0.2393739f, 0.73558828f, 0.45568838f},
    {0.24606968f, 0.73890972f, 0.45202405f},
    {0.25289851f, 0.74221104f, 0.44828355f},
    {0.25985676f, 0.74549162f, 0.44446673f},
    {0.26694127f, 0.74875084f, 0.44057284f},
    {0.27414922f, 0.75198807f, 0.4366009f},
    {0.28147681f, 0.75520266f, 0.43255207f},
    {0.28892102f, 0.75839399f, 0.42842626f},
    {0.29647899f, 0.76156142f, 0.42422341f},
    {0.30414796f, 0.76470433f, 0.41994346f},
    {0.31192534f, 0.76782207f, 0.41558638f},
    {0.3198086f, 0.77091403f, 0.41115215f},
    {0.3277958f, 0.77397953f, 0.40664011f},
    {0.33588539f, 0.7770179f, 0.40204917f},
    {0.34407411f, 0.78002855f, 0.39738103f},
    {0.35235985f, 0.78301086f, 0.39263579f},
    {0.36074053f, 0.78596419f, 0.38781353f},
    {0.3692142f, 0.78888793f, 0.38291438f},
    {0.37777892f, 0.79178146f, 0.3779385f},
    {0.38643282f, 0.79464415f, 0.37288606f},
    {0.39517408f, 0.79747541f, 0.36775726f},
    {0.40400101f, 0.80027461f, 0.36255223f},
    {0.4129135f, 0.80304099f, 0.35726893f},
    {0.42190813f, 0.80577412f, 0.35191009f},
    {0.43098317f, 0.80847343f, 0.34647607f},
    {0.44013691f, 0.81113836f, 0.3409673f},
    {0.44936763f, 0.81376835f, 0.33538426f},
    {0.45867362f, 0.81636288f, 0.32972749f},
    {0.46805314f, 0.81892143f, 0.32399761f},
    {0.47750446f, 0.82144351f, 0.31819529f},
    {0.4870258f, 0.82392862f, 0.31232133f},
    {0.49661536f, 0.82637633f, 0.30637661f},
    {0.5062713f, 0.82878621f, 0.30036211f},
    {0.51599182f, 0.83115784f, 0.29427888f},
    {0.52577622f, 0.83349064f, 0.2881265f},
    {0.5356211f, 0.83578452f, 0.28190832f},
    {0.5455244f, 0.83803918f, 0.27562602f},
    {0.55548397f, 0.84025437f, 0.26928147f},
    {0.5654976f, 0.8424299f, 0.26287683f},
    {0.57556297f, 0.84456561f, 0.25641457f},
    {0.58567772f, 0.84666139f, 0.24989748f},
    {0.59583934f, 0.84871722f, 0.24332878f},
    {0.60604528f, 0.8507331f, 0.23671214f},
    {0.61629283f, 0.85270912f, 0.23005179f},
    {0.62657923f, 0.85464543f, 0.22335258f},
    {0.63690157f, 0.85654226f, 0.21662012f},
    {0.64725685f, 0.85839991f, 0.20986086f},
    {0.65764197f, 0.86021878f, 0.20308229f},
    {0.66805369f, 0.86199932f, 0.19629307f},
    {0.67848868f, 0.86374211f, 0.18950326f},
    {0.68894351f, 0.86544779f, 0.18272455f},
    {0.69941463f, 0.86711711f, 0.17597055f},
    {0.70989842f, 0.86875092f, 0.16925712f},
    {0.72039115f, 0.87035015f, 0.16260273f},
    {0.73088902f, 0.87191584f, 0.15602894f},
    {0.74138803f, 0.87344918f, 0.14956101f},
    {0.75188414f, 0.87495143f, 0.14322828f},
    {0.76237342f, 0.87642392f, 0.13706449f},
    {0.77285183f, 0.87786808f, 0.13110864f},
    {0.78331535f, 0.87928545f, 0.12540538f},
    {0.79375994f, 0.88067763f, 0.12000532f},
    {0.80418159f, 0.88204632f, 0.11496505f},
    {0.81457634f, 0.88339329f, 0.11034678f},
    {0.82494028f, 0.88472036f, 0.10621724f},
    {0.83526959f, 0.88602943f, 0.1026459f},
    {0.84556056f, 0.88732243f, 0.09970219f},
    {0.8558096f, 0.88860134f, 0.09745186f},
    {0.86601325f, 0.88986815f, 0.09595277f},
    {0.87616824f, 0.89112487f, 0.09525046f},
    {0.88627146f, 0.89237353f, 0.09537439f},
    {0.89632002f, 0.89361614f, 0.09633538f},
    {0.90631121f, 0.89485467f, 0.09812496f},
    {0.91624212f, 0.89609127f, 0.1007168f},
    {0.92610579f, 0.89732977f, 0.10407067f},
    {0.93590444f, 0.8985704f, 0.10813094f},
    {0.94563626f, 0.899815f, 0.11283773f},
    {0.95529972f, 0.90106534f, 0.11812832f},
    {0.96489353f, 0.90232311f, 0.12394051f},
    {0.97441665f, 0.90358991f, 0.13021494f},
    {0.98386829f, 0.90486726f, 0.13689671f},
    {0.99324789f, 0.90615657f, 0.1439362f}};

Eigen::Array<double, -1, 1> compute_dark_count(
    const Eigen::Ref<Eigen::Array<double, -1, -1, Eigen::RowMajor>>& image) {
    const size_t image_h = image.rows();
    const size_t image_w = image.cols();

    Eigen::Array<double, -1, 1> tmp{image_w};
    Eigen::Array<double, -1, 1> new_dark_count{image_h};

    // probably computed lazily when used below?
    auto row_diffs = image.bottomRows(image_h - 1) - image.topRows(image_h - 1);

    // to handle azimuth-masked data, only consider columns with nonzero values
    Eigen::Array<bool, -1, 1> col_mask =
        image.template cast<bool>().colwise().any();
    const size_t n_cols = col_mask.count();

    if (n_cols == 0) {
        new_dark_count.setZero(image_h);
        return new_dark_count;
    }

    Eigen::Array<double, -1, -1, Eigen::RowMajor> row_diffs_nonzero{image_h - 1, n_cols};
    for (size_t i = 0, j = 0; i < image_w && j < n_cols; i++) {
        if (col_mask[i]) {
            row_diffs_nonzero.col(j) = row_diffs.col(i);
            j++;
        }
    }

    // compute the median of differences between rows
    for (size_t i = 1; i < image_h; i++) {
        tmp = row_diffs_nonzero.row(i - 1);
        std::nth_element(tmp.data(), tmp.data() + n_cols / 2,
                         tmp.data() + n_cols);
        new_dark_count[i] = new_dark_count[i - 1] + tmp[n_cols / 2];
    }

    // remove gradients in the entire height of image by doing linear fit
    Eigen::Matrix<double, -1, 2> A(image_h, 2);
    for (size_t i = 0; i < image_h; i++) {
        A(i, 0) = 1;
        A(i, 1) = static_cast<double>(i);
    }
    Eigen::Matrix<double, 2, 1> x = A.fullPivLu().solve(new_dark_count.matrix());
    new_dark_count -= (A * x).array();

    // subtract minimum value
    new_dark_count -= new_dark_count.minCoeff();
    return new_dark_count;
}

void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& point_msg)
{
    //首先将接收到的ROS格式的点云消息sensor_msgs::PointCloud2转化为PCL格式的消息    PointCloud<pcl::PointXYZITR>
    //根据雷达型号，创建Mat矩阵，由于在此使用的雷达为128线，每条线上有1250个点，所以创建了一个大小为128*1250尺寸的矩阵，并用0元素初始化。   
    pcl::PointCloud<pcl::PointXYZITR> cloud_msg;
    pcl::fromROSMsg(*point_msg, cloud_msg);
    float range_depth; //range为该点的深度值
    //float range_rgb; //need to use size_t
    int row_i, col_i; 
    int scan_num = 128;
    int horizon_num = 1250;
    //const float PI = 3.1415926;
    const float ANG = 57.2957795; //180/pi

    //depth
    sensor_msgs::Image range_image ;
    range_image.width = horizon_num;
        range_image.height = scan_num;
        range_image.step = horizon_num*sizeof(uint16_t);
        range_image.encoding = sensor_msgs::image_encodings::MONO16;
        range_image.data.resize(horizon_num * scan_num * sizeof(uint16_t));
        range_image.header.stamp = point_msg->header.stamp;
        range_image.header.frame_id = point_msg->header.frame_id;

    auto range_image_map_uint = Eigen::Map<Eigen::Array<uint16_t, -1, -1, Eigen::RowMajor>>(
            (uint16_t*)range_image.data.data(), scan_num, horizon_num);
    Eigen::Array<double, -1, -1, Eigen::RowMajor> range_image_map = range_image_map_uint.cast<double>();
    //rgb
    sensor_msgs::Image range_image_rgb;
    range_image_rgb.width = horizon_num;
    range_image_rgb.height = scan_num;
    range_image_rgb.step = horizon_num *3;
    range_image_rgb.encoding = "rgb8";
    range_image_rgb.data.resize(horizon_num * scan_num * 3);
    range_image_rgb.header.stamp = point_msg->header.stamp;
    range_image_rgb.header.frame_id = point_msg->header.frame_id;

    auto range_image_map_rgb = Eigen::Map<Eigen::Array<uint8_t, -1, -1, Eigen::RowMajor>>(
            (uint8_t*)range_image_rgb.data.data(), scan_num, horizon_num*3);

    for(const auto pt : cloud_msg.points)
    {
        if(pt.ring != 0)  //忽略NAN点(NAN点的ring值为0)
        {
            //row_i = pt.ring;
            //col_i = (62.5 - atan2(pt.y, pt.x) * ANG) / 0.1;

            //row_i don't use ring but calculation
            float xyz_range = (float)sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            row_i = (12.5 -(90- (atan2(xyz_range, pt.z) * ANG))) /0.2; 
            col_i = (62.5 - atan2(pt.y, pt.x) * ANG) / 0.1;
            if(row_i >128){row_i=128;}
            if(row_i <0){row_i =0;}

            //depth
            range_depth = pt.range * 60000 / 10000;
            //range_depth = pt.range;

            //rgb
            size_t range_rgb = pt.range * 255.0 / 10000;
            if(range_rgb >=255){
                range_rgb = 255;
            }

            //忽略索引不合法的点
            if(row_i < 0 || row_i >= scan_num)
                continue;
            if(col_i < 0 || col_i >= horizon_num)
                continue;
            //depth
            range_image_map(row_i, col_i) = range_depth > pixel_value_max ? 0 : range_depth;
            //rgb
            range_image_map_rgb(row_i, col_i *3) = static_cast<uint8_t>(viridis_palette[range_rgb][0]  *255);
            range_image_map_rgb(row_i, col_i *3 +1) =static_cast<uint8_t>(viridis_palette[range_rgb][1] *255);
            range_image_map_rgb(row_i, col_i *3 +2) =static_cast<uint8_t>(viridis_palette[range_rgb][2]*255);
        }
    }


    //auto exposure
    bool AutoExposureGray =0;
    bool AutoExposureRGB  =0;


    if(AutoExposureGray)
    {
        bool update_state=true;
        static int counter = 0;

        double lo_percentile, hi_percentile;  // percentiles used for scaling
        int ae_update_every;

        double lo_state = -1.0;
        double hi_state = -1.0;
        double lo = -1.0;
        double hi = -1.0;

        int MappingRange = 50000;

        bool initialized = false;
    
        //damping makes the autoexposure smooth and avoids flickering 
        const double ae_damping =0.9;

        const int ae_default_update_every =1;
        /* for performance reasons, only consider a subset of points */
        const size_t ae_stride = 4;

        /* if there are too few points, do nothing */
        const size_t ae_min_nonzero_points = 100;

        /* default percentile for scaling in autoexposure */
        const double lo_default_percentile = 0.1;
        const double hi_default_percentile = 0.2;

        lo_percentile=lo_default_percentile;
        hi_percentile=hi_default_percentile;

        ae_update_every=ae_default_update_every;

        Eigen::Ref<Eigen::Array<double, -1, -1, Eigen::RowMajor>> image = range_image_map;
        Eigen::Map<Eigen::Array<double, -1, 1>> key_eigen(image.data(), image.size());

        if (counter == 0 &&update_state) {
            const size_t n = key_eigen.rows();
            std::vector<size_t> indices;
            indices.reserve(n);
            for (size_t i = 0; i < n; i += ae_stride) {
                // ignore 0 values, which are often due to dropped packets etc
                if (key_eigen[i] > 0) {
                    indices.push_back(i);
                }
            }
            if (indices.size() < ae_min_nonzero_points) {
                // too few nonzero values, nothing to do
                return;
            }

            auto cmp = [&](const size_t a, const size_t b) {
                return key_eigen(a) < key_eigen(b);
            };

            const size_t lo_kth_extreme = static_cast<size_t>(indices.size() * lo_percentile);
            std::nth_element(indices.begin(), indices.begin() + lo_kth_extreme, indices.end(), cmp);
            lo = key_eigen[*(indices.begin() + lo_kth_extreme)];

            const size_t hi_kth_extreme = static_cast<size_t>(indices.size() * hi_percentile);
            std::nth_element(indices.begin() + lo_kth_extreme,
                         indices.end() - hi_kth_extreme - 1, indices.end(),
                         cmp);
            hi = key_eigen[*(indices.end() - hi_kth_extreme - 1)];

            if (!initialized) {
                initialized = true;
                lo_state = lo;
                hi_state = hi;
            }
        }
        if (!initialized) {
            return;
        }

        // we use the simplest form of exponential smoothing
        if (update_state) {
            lo_state = ae_damping * lo_state + (1.0 - ae_damping) * lo;
            hi_state = ae_damping * hi_state + (1.0 - ae_damping) * hi;
        }

        // Apply affine transformation mapping lo_state to lo_percentile and
        // hi_state to 1 - hi_percentile. If it would map 0 to positive number,
        // instead map using only hi_state
        double lo_hi_scale =
            ((1.0 - (lo_percentile + hi_percentile)) / (hi_state - lo_state)) *MappingRange;
        //std::cout<< "lo_hi_scale:"<<lo_hi_scale<<std::endl;
        //std::cout<< "hi_state   :"<<hi_state<<std::endl;
        //std::cout<< "lo_state   :"<<lo_state<<std::endl;

        if (std::isinf(lo_hi_scale) || std::isnan(lo_hi_scale)) {
            // map everything relative to hi_state being 0.5 due to small spread or nan
            key_eigen *= 0.5 / hi_state;
        } else if (lo_hi_scale * (0.0 - lo_state) + lo_percentile <= 0.00) {
            // apply affine transformation
            key_eigen -= lo_state;
            key_eigen *= lo_hi_scale;
            //key_eigen += lo_percentile
        } else {
            // lo_hi_state transformation would map 0 to positive number
            // instead, map using only hi_state
            key_eigen *= (1.0 - hi_percentile) / (hi_state);
        }

        // clamp
        //key_eigen = key_eigen.max(0.0).min(1.0);

        if (update_state) {
        counter = (counter + 1) % ae_update_every;
        }


        //BeamUniformityCorrector
        static int counter_Beam = 0;
        double buc_damping =0.92;
        int buc_update_every =8;

        static Eigen::ArrayXd dark_count;
        Eigen::Ref<Eigen::Array<double, -1, -1, Eigen::RowMajor>> image_Beam = range_image_map;

        const auto image_h = image_Beam.rows();

        // compute dark counts, if necessary
        if (dark_count.size() != image_h) {
            dark_count = compute_dark_count(image_Beam).template cast<double>();
        } else if (update_state && counter_Beam == 0) {
            // if previous state exists, update using exponential smoothing:
            Eigen::ArrayXd new_dark_count =
                compute_dark_count(image_Beam).template cast<double>();

            dark_count *= buc_damping;
            dark_count += new_dark_count * (1.0 - buc_damping);
        }
        counter_Beam = (counter_Beam + 1) % buc_update_every;

        // apply the dark count correction
        image_Beam.colwise() -= dark_count.cast<double>();

        // clamp any negative values
        image_Beam = image_Beam.cwiseMax(0.0);
    }        

    Eigen::Array<uint16_t, -1, -1, Eigen::RowMajor> range_image_map_tmp =range_image_map.cast<uint16_t>();
    std::memcpy(range_image_map_uint.data(),range_image_map_tmp.data(),range_image_map_tmp.size()*sizeof(uint16_t));

    //auto exposure rgb
    if(AutoExposureRGB){
        for(int row_tmp =0;row_tmp < range_image_map.rows(); ++row_tmp){
            for(int col_tmp =0;col_tmp < range_image_map.cols(); ++col_tmp){
                size_t range_rgb = static_cast<uint8_t>(range_image_map(row_tmp, col_tmp) * 255.0 / 60000);
                if(range_rgb >=255){
                    range_rgb = 255;
                }
                //rgb
                range_image_map_rgb(row_tmp, col_tmp *3) = 
                        static_cast<uint8_t>(viridis_palette[range_rgb][0] *255);
                range_image_map_rgb(row_tmp, col_tmp *3 +1) =
                        static_cast<uint8_t>(viridis_palette[range_rgb][1] *255);
                range_image_map_rgb(row_tmp, col_tmp *3 +2) =
                        static_cast<uint8_t>(viridis_palette[range_rgb][2] *255);
            }
        }
    }

    pub.publish(range_image);
    pub_rgb.publish(range_image_rgb);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "depth_make_node");
    ros::NodeHandle nh;

    pub     = nh.advertise<sensor_msgs::Image>("/ag_points",     100);
    pub_rgb = nh.advertise<sensor_msgs::Image>("/ag_points_rgb", 100);

    ros::Subscriber mapSub = nh.subscribe("/aglidar_points", 100, PointCallback);

    ros::spin();
    return 0;
}
