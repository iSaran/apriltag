#include <iostream>

#include <fstream>

#include <opencv2/opencv.hpp>

#include "boost/filesystem.hpp"
#include <boost/format.hpp>

#include <apriltag.h>
#include <tag36h11.h>
#include <ros/ros.h>
#include <ros/package.h>


void makePattern36H11(const std::string &tagFolder, const std::string &outSvg, const cv::Size &boardSize, float tileSize, float tileOffset)
{
    using namespace boost::filesystem ;

    float tx = boardSize.width * (tileSize + tileOffset) ;
    float ty = boardSize.height * (tileSize + tileOffset) ;

    float bs = tileSize/8 ;

    std::ofstream strm(outSvg.c_str()) ;

    strm << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"" ;
    strm << tx*100 << "cm\" height=\"" << ty*100 << "cm\" viewBox=\"0 0 " ;
    strm << tx << ' ' << ty << "\">\n<g fill=\"black\">" ;

    for(int i=0, k=0 ; i<boardSize.height ; i++)
        for(int j=0 ; j<boardSize.width ; j++, k++)
        {
            path dir(tagFolder) ;

            path p = dir / str(boost::format("tag36_11_%05d.png") % k) ;

            if ( !exists(p) ) continue ;

            cv::Mat cc = cv::imread(p.string()) ;

            float x0 = j * (tileSize + tileOffset) ;
            float y0 = i * (tileSize + tileOffset) ;

            strm << "<g>\n" ;
            for( int y=0 ; y<8 ; y++)
                for( int x=0 ; x<8 ; x++)
                {

                    if ( cc.at<cv::Vec3b>(y+1, x+1)[0] == 0 )
                    {
                        float xx = x0 + x * bs ;
                        float yy = y0 + y * bs ;

                        strm << "<rect x=\"" << xx << "\" y=\"" << yy << "\" width=\"" << bs << "\" height=\"" << bs << "\"/>\n" ;
                    }
                }
            strm << "</g>\n" ;
        }

    strm << "</g></svg>" ;

    strm.flush() ;
}


int main(int argc, char** argv)
{
    // std::string package_path = ;

    ros::init(argc, argv, "make_pattern");
    ros::NodeHandle n;

    std::string pkg_path = ros::package::getPath("apriltag_detector");
    std::string tag_path = pkg_path + "/3rdparty/apriltag/tag36h11/";
    std::string svg_path = pkg_path + "/output/pattern.svg";

    std::vector<int> board_size(2);
    n.getParam("apriltag_detector_params/make_pattern/board_size", board_size);

    double tile_size;
    n.getParam("apriltag_detector_params/make_pattern/tile_size", tile_size);

    double tile_offset;
    n.getParam("apriltag_detector_params/make_pattern/tile_offset", tile_offset);
    
    ROS_INFO_STREAM("Creating pattern with:");
    ROS_INFO_STREAM("path: " << svg_path);
    ROS_INFO_STREAM("board size: " << board_size.at(0) << ", " << board_size.at(1));
    ROS_INFO_STREAM("tile size: " << tile_size);
    ROS_INFO_STREAM("tile offset: " << tile_offset);

    makePattern36H11(tag_path, svg_path, cv::Size(board_size.at(0), board_size.at(1)), tile_size, tile_offset); 
    return 0;
}
