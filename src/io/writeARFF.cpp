/*
 *  Copyright (c) 2017  Rodolfo Lotte (lotte@dsr.inpe.br - rodolfo.lotte@gmail.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERRGBANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "superpixel.hpp"

using namespace std;

void writeARFF(const char *OutFilename,        
              size_t m_labels,                            
              const u_int32_t m_bands,
              const u_int32_t entropy_bands,                                
              const u_int32_t hsv_bands,
              const u_int32_t lab_bands, 
              const cv::Mat avgRGB,
              const cv::Mat stdRGB,
              const cv::Mat varRGB,              
              const cv::Mat avgEntropy,
              const cv::Mat stdEntropy,
              const cv::Mat varEntropy,
              const cv::Mat avgHSV,
              const cv::Mat stdHSV,
              const cv::Mat varHSV,              
              const cv::Mat avgLab,
              const cv::Mat stdLab,
              const cv::Mat varLab,              
              const std::vector<double> area,
              const std::vector<double> perimeter,                  
              const std::vector<double> width,
              const std::vector<double> height,
              const std::vector<double> elongation,
              const std::vector<double> circularity,
              const std::vector<double> rectangularity,
              const std::vector<double> compactness )
{
  
  ofstream arffSegments;
  string filename_no_extension = remove_extension(OutFilename);
  string filename = filename_no_extension + ".arff";
  arffSegments.open(filename.c_str(), fstream::in | fstream::out | fstream::trunc);    
  
  std::string headerAvg, headerStd, headerVar;
  stringstream value;
  
  // relation
  arffSegments << "@relation " + filename + "\n\n";

  // rgb
  for (size_t b = 0; b < m_bands; b++)
  {    
    value << b + 1;
    headerAvg = "@attribute ' " + value.str() + "_avg' numeric\n";
    arffSegments << headerAvg;

    headerStd = "@attribute ' " + value.str() + "_std' numeric\n";
    arffSegments << headerStd;

    headerVar = "@attribute ' " + value.str() + "_var' numeric\n";
    arffSegments << headerVar;    
    
    value.str("");
  }

  // local entropy
  for (size_t b = 0; b < entropy_bands; b++)
  {      
    value << b + 1;
    headerAvg = "@attribute ' " + value.str() + "_e_avg' numeric\n";
    arffSegments << headerAvg;

    headerStd = "@attribute ' " + value.str() + "_e_std' numeric\n";
    arffSegments << headerStd;

    headerVar = "@attribute ' " + value.str() + "_e_var' numeric\n";
    arffSegments << headerVar;    

    value.str("");
  }

  // hsv
  for (size_t b = 0; b < hsv_bands; b++)
  {      
    value << b + 1;
    headerAvg = "@attribute ' " + value.str() + "_hsv_avg' numeric\n";
    arffSegments << headerAvg;

    headerStd = "@attribute ' " + value.str() + "_hsv_std' numeric\n";
    arffSegments << headerStd;

    headerVar = "@attribute ' " + value.str() + "_hsv_var' numeric\n";
    arffSegments << headerVar;    

    value.str("");
  }

  // lab
  for (size_t b = 0; b < lab_bands; b++)
  {      
    value << b + 1;
    headerAvg = "@attribute ' " + value.str() + "_lab_avg' numeric\n";
    arffSegments << headerAvg;

    headerStd = "@attribute ' " + value.str() + "_lab_std' numeric\n";
    arffSegments << headerStd;

    headerVar = "@attribute ' " + value.str() + "_lab_var' numeric\n";
    arffSegments << headerVar;    

    value.str("");
  }

  // geometries
  arffSegments << "@attribute ' area' numeric\n"; 
  arffSegments << "@attribute ' perimeter' numeric\n";
  arffSegments << "@attribute ' width' numeric\n";
  arffSegments << "@attribute ' height' numeric\n";
  arffSegments << "@attribute ' elongation' numeric\n";
  arffSegments << "@attribute ' circularity' numeric\n";
  arffSegments << "@attribute ' rectang' numeric\n";
  arffSegments << "@attribute ' compactness' numeric\n";
  arffSegments << "@attribute ' class' {sky,balcony,wall,other,window,roof}\n";
  arffSegments <<"\n@data\n";
  
  
  for (size_t k = 0; k < m_labels; k++)
  {    
    for(u_int32_t avg=0; avg < m_bands; avg++)
    {
      arffSegments << avgRGB.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < m_bands; std++)
    {
      arffSegments << stdRGB.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < m_bands; var++)
    {
      arffSegments << varRGB.at<double>(var,k) << ", ";
    }
    
    for(u_int32_t avg=0; avg < entropy_bands; avg++)
    {
      arffSegments << avgEntropy.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < entropy_bands; std++)
    {
      arffSegments << stdEntropy.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < entropy_bands; var++)
    {
      arffSegments << varEntropy.at<double>(var,k) << ", ";
    }

    for(u_int32_t avg=0; avg < hsv_bands; avg++)
    {
      arffSegments << avgHSV.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < hsv_bands; std++)
    {
      arffSegments << stdHSV.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < hsv_bands; var++)
    {
      arffSegments << varHSV.at<double>(var,k) << ", ";
    }

    for(u_int32_t avg=0; avg < lab_bands; avg++)
    {
      arffSegments << avgLab.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < lab_bands; std++)
    {
      arffSegments << stdLab.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < lab_bands; var++)
    {
      arffSegments << varLab.at<double>(var,k) << ", ";
    }
    arffSegments << area[k] << ", " << perimeter[k] << ", " << width[k] << ", " << height[k] << ", " << elongation[k] << ", " << circularity[k] << ", " << rectangularity[k] << ", " << compactness[k] << ", " << "?\n";
  }

  arffSegments.close();  
}

 

