/*
 *  Copyright (c) 2015  Balint Cristian (cristian.balint@gmail.com)
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

/* vector.cpp */
/* Vector I/O */

#include <omp.h>

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "cpl_string.h"
#include "cpl_csv.h"

#include <opencv2/opencv.hpp>

#include "superpixel.hpp"

using namespace std;
using namespace cv;

void LabelContours(const cv::Mat klabels, std::vector<std::vector<LINE> > &linelists)
{
  // iterate through pixels and RGBeck edges
  printf("Parse edges in segmented image\n");
  printf("klabels rows: %d\n", klabels.rows);
  printf("klabels cols: %d\n", klabels.cols);

  for (int y = 0; y < klabels.rows; y++)
  {
    const int yklabels = y * klabels.cols;
    for (int x = 0; x < klabels.cols; x++)
    {
      const int i = yklabels + x;

      LINE line;
      // RGBeck right pixel
      if (x == klabels.cols - 1)
      {
        line.sX = x + 1;
        line.sY = y;
        line.eX = x + 1;
        line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      else if (klabels.at<u_int32_t>(i) != klabels.at<u_int32_t>(i + 1))
      {
        line.sX = x + 1;
        line.sY = y;
        line.eX = x + 1;
        line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      // RGBeck left pixel
      if (x == 0)
      {
        line.sX = x;
        line.sY = y;
        line.eX = x, line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      else if (klabels.at<u_int32_t>(i) != klabels.at<u_int32_t>(i - 1))
      {
        line.sX = x;
        line.sY = y;
        line.eX = x, line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      // RGBeck top pixel
      if (y == 0)
      {
        line.sX = x;
        line.sY = y;
        line.eX = x + 1;
        line.eY = y;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      else if (klabels.at<u_int32_t>(i) != klabels.at<u_int32_t>((y - 1) * klabels.cols + x))
      {
        line.sX = x;
        line.sY = y;
        line.eX = x + 1;
        line.eY = y;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      // RGBeck bottom pixel
      if (y == klabels.rows - 1)
      {
        line.sX = x;
        line.sY = y + 1;
        line.eX = x + 1;
        line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
      else if (klabels.at<u_int32_t>(i) != klabels.at<u_int32_t>((y + 1) * klabels.cols + x))
      {
        line.sX = x;
        line.sY = y + 1;
        line.eX = x + 1;
        line.eY = y + 1;
        {
          linelists[klabels.at<u_int32_t>(i)].push_back(line);
        }
      }
    }
    GDALTermProgress((float)(y) / (float)(klabels.rows), NULL, NULL);
  }
  GDALTermProgress(1.0f, NULL, NULL);
}

double getMax(double currentMax, double value)
{
  if (value > currentMax)
  {
    currentMax = value;
  }
  return currentMax;
}

double getMin(double currentMin, double value)
{
  if (value < currentMin)
  {
    currentMin = value;
  }
  return currentMin;
}

void ComputeGeometries(size_t m_labels,
                       cv::Mat &labelpixels,
                       std::vector<double> &area,
                       std::vector<double> &perimeter,
                       std::vector<double> &width,
                       std::vector<double> &height,
                       std::vector<double> &elongation,
                       std::vector<double> &circularity,
                       std::vector<double> &rectangularity,
                       std::vector<double> &compactness,
                       std::vector<std::vector<LINE> > &linelists)
{
  printf("Compute Geometries\n");
  printf("       ");

  double per = 0.0;
  double major_axis = 0.0;
  double minor_axis = 0.0;
  double area_bbox = 0.0;
  double xmax = 0.0;
  double xmin = 999999.0;
  double ymax = 0.0;
  double ymin = 999999.0;

  // printf ("mLabels: %lu - lineLists: %lu\n", m_labels, linelists.size());

  for (size_t k = 0; k < m_labels; k++)
  {

    // printf ("-- lineLists[%lu]: %lu\n", k, linelists[k].size());

    if (linelists[k].size() > 0)
    {
      per = 0.0;
      xmax = 0.0;
      xmin = 999999.0;
      ymax = 0.0;
      ymin = 999999.0;

      for (unsigned j = 0; j < linelists[k].size(); j++)
      {
        per += sqrt(pow(linelists[k][j].eX - linelists[k][j].sX, 2) + pow(linelists[k][j].eY - linelists[k][j].sY, 2));

        if ((j + 1) != linelists[k].size())
        {
          xmax = getMax(xmax, linelists[k][j].sX);
          xmin = getMin(xmin, linelists[k][j].sX);
          ymax = getMax(ymax, linelists[k][j].sY);
          ymin = getMin(ymin, linelists[k][j].sY);
        }
        else
        {
          xmax = getMax(xmax, linelists[k][j].eX);
          xmin = getMin(xmin, linelists[k][j].eX);
          ymax = getMax(ymax, linelists[k][j].eY);
          ymin = getMin(ymin, linelists[k][j].eY);
        }
      }

      area[k] = (double) labelpixels.at<u_int32_t>(k);
      perimeter[k] = per;
      width[k] = (xmax - xmin);
      height[k] = (ymax - ymin);

      if (width[k] > height[k])
      {
        major_axis = width[k];
        minor_axis = height[k];
      }
      else
      {
        minor_axis = width[k];
        major_axis = height[k];
      }
      elongation[k] = major_axis / minor_axis;

      circularity[k] = 4 * area[k] / pow(perimeter[k], 2);

      area_bbox = width[k] * height[k];
      rectangularity[k] = area[k] / area_bbox;

      compactness[k] = 2 * sqrt(M_PI * area[k] / perimeter[k]);
    }
  }
}

void SavePolygons( const std::vector< std::string > InFilenames,
                    const char *OutFilename, 
                    const char *OutFormat,                   
                    const u_int32_t m_bands,
                    const u_int32_t entropy_bands,
                    const u_int32_t hsv_bands,
                    const u_int32_t lab_bands,
                    const cv::Mat labelpixels,                   
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
                    const std::vector<double> compactness,
                    std::vector< std::vector< LINE > >& linelists )
{

  CPLLocaleC oLocaleCForcer();
  CPLErrorReset();

  #if GDALVER >= 2
  GDALDriver *liDriver;
  liDriver = GetGDALDriverManager()->GetDriverByName(OutFormat);
  #else
  OGRSFDriver *liDriver;
  liDriver = OGRSFDriverRegistrar::GetRegistrar()
                 ->GetDriverByName(OutFormat);
  #endif

  if (liDriver == NULL)
  {
    printf("\nERROR: %s driver not available.\n", OutFormat);
    exit(1);
  }

  #if GDALVER >= 2
  GDALDataset *liDS;
  liDS = liDriver->Create(OutFilename, 0, 0, 0, GDT_Unknown, NULL);
  #else
  OGRDataSource *liDS;
  liDS = liDriver->CreateDataSource(OutFilename, NULL);
  #endif

  if (liDS == NULL)
  {
    printf("\nERROR: Creation of output file failed.\n");
    exit(1);
  }

  const size_t m_labels = labelpixels.rows;

  // dataset
  GDALDataset *piDataset;
  piDataset = (GDALDataset *)GDALOpen(InFilenames[0].c_str(), GA_ReadOnly);

  // spatialref
  OGRSpatialReference oSRS;
  oSRS.SetProjCS(piDataset->GetProjectionRef());

  OGRLayer *liLayer;
  liLayer = liDS->CreateLayer(InFilenames[0].c_str(), &oSRS, wkbPolygon, NULL);

  if (liLayer == NULL)
  {
    printf("\nERROR: Layer creation failed.\n");
    exit(1);
  }

  // spatial transform
  double adfGeoTransform[6];
  double oX = 0.0f;
  double oY = 0.0f;
  double mX = 1.0f;
  double mY = -1.0f;
  if (piDataset->GetGeoTransform(adfGeoTransform) == CE_None)
  {
    oX = adfGeoTransform[0];
    oY = adfGeoTransform[3];
    mX = adfGeoTransform[1];
    mY = adfGeoTransform[5];
  }
  GDALClose((GDALDatasetH)piDataset);

  stringstream value;
  std::string FieldName;

  OGRFieldDefn *clsIdField = new OGRFieldDefn("ID", OFTInteger);
  liLayer->CreateField(clsIdField);

  for (size_t b = 0; b < m_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_AVG";
    OGRFieldDefn *lavrgField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lavrgField);
    value.str("");
  }

  for (size_t b = 0; b < m_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_STD";
    OGRFieldDefn *lstdField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lstdField);
    value.str("");
  }

  for (size_t b = 0; b < m_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_VAR";
    OGRFieldDefn *lvarField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lvarField);
    value.str("");
  }

  for (size_t b = 0; b < entropy_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_AVG_E";
    OGRFieldDefn *lavrgField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lavrgField);
    value.str("");
  }

  for (size_t b = 0; b < entropy_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_STD_E";
    OGRFieldDefn *lstdField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lstdField);
    value.str("");
  }

  for (size_t b = 0; b < entropy_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_VAR_E";
    OGRFieldDefn *lvarField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lvarField);
    value.str("");
  }

  for (size_t b = 0; b < hsv_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_AVG_HSV";
    OGRFieldDefn *lavrgField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lavrgField);
    value.str("");
  }

  for (size_t b = 0; b < hsv_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_STD_HSV";
    OGRFieldDefn *lstdField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lstdField);
    value.str("");
  }

  for (size_t b = 0; b < hsv_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_VAR_HSV";
    OGRFieldDefn *lvarField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lvarField);
    value.str("");
  }

  for (size_t b = 0; b < lab_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_AVG_LAB";
    OGRFieldDefn *lavrgField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lavrgField);
    value.str("");
  }

  for (size_t b = 0; b < lab_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_STD_LAB";
    OGRFieldDefn *lstdField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lstdField);
    value.str("");
  }

  for (size_t b = 0; b < lab_bands; b++)
  {
    value << b + 1;
    FieldName = value.str() + "_VAR_LAB";
    OGRFieldDefn *lvarField = new OGRFieldDefn(FieldName.c_str(), OFTReal);
    liLayer->CreateField(lvarField);
    value.str("");
  }

  OGRFieldDefn *pixArField = new OGRFieldDefn("AREA", OFTReal);
  liLayer->CreateField(pixArField);
  OGRFieldDefn *pixPerField = new OGRFieldDefn("PERIMETER", OFTReal);
  liLayer->CreateField(pixPerField);
  OGRFieldDefn *pixWidthField = new OGRFieldDefn("WIDTH", OFTReal);
  liLayer->CreateField(pixWidthField);
  OGRFieldDefn *pixHeightField = new OGRFieldDefn("HEIGHT", OFTReal);
  liLayer->CreateField(pixHeightField);
  OGRFieldDefn *pixElongField = new OGRFieldDefn("ELONG", OFTReal);
  liLayer->CreateField(pixElongField);
  OGRFieldDefn *pixCircField = new OGRFieldDefn("CIRC", OFTReal);
  liLayer->CreateField(pixCircField);
  OGRFieldDefn *pixRectField = new OGRFieldDefn("RECT", OFTReal);
  liLayer->CreateField(pixRectField);
  OGRFieldDefn *pixComptField = new OGRFieldDefn("COMPAC", OFTReal);
  liLayer->CreateField(pixComptField);
  OGRFieldDefn *clsClassField = new OGRFieldDefn("CLASS", OFTString);
  liLayer->CreateField(clsClassField);

  OGRFeature *liFeature;
  int multiring = 0;  
  printf("Write File: %s (polygon)\n", OutFilename);
  for (size_t k = 0; k < m_labels; k++)
  {
    liFeature = OGRFeature::CreateFeature(liLayer->GetLayerDefn());

    if (multiring == 1)
    {
      multiring = 0;      
      k -= 1;      
    }

    if (linelists[k].size() != 0)
    {
      liFeature->SetField("ID", (int)k);

      for (size_t b = 0; b < m_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_AVG";
        liFeature->SetField(FieldName.c_str(), (double)avgRGB.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < m_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_STD";
        liFeature->SetField(FieldName.c_str(), (double)stdRGB.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < m_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_VAR";
        liFeature->SetField(FieldName.c_str(), (double)varRGB.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < entropy_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_AVG_E";
        liFeature->SetField(FieldName.c_str(), (double)avgEntropy.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < entropy_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_STD_E";
        liFeature->SetField(FieldName.c_str(), (double)stdEntropy.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < entropy_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_VAR_E";
        liFeature->SetField(FieldName.c_str(), (double)varEntropy.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < hsv_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_AVG_HSV";
        liFeature->SetField(FieldName.c_str(), (double)avgHSV.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < hsv_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_STD_HSV";
        liFeature->SetField(FieldName.c_str(), (double)stdHSV.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < hsv_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_VAR_HSV";
        liFeature->SetField(FieldName.c_str(), (double)varHSV.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < lab_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_AVG_LAB";
        liFeature->SetField(FieldName.c_str(), (double)avgLab.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < lab_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_STD_LAB";
        liFeature->SetField(FieldName.c_str(), (double)stdLab.at<double>(b,k));
        value.str("");
      }
      for (size_t b = 0; b < lab_bands; b++)
      {
        value << b + 1;
        FieldName = value.str() + "_VAR_LAB";
        liFeature->SetField(FieldName.c_str(), (double)varLab.at<double>(b,k));
        value.str("");
      }


      liFeature->SetField("AREA", (double)area[k]);
      liFeature->SetField("PERIMETER", (double)perimeter[k]);
      liFeature->SetField("WIDTH", (double)width[k]);
      liFeature->SetField("HEIGHT", (double)height[k]);
      liFeature->SetField("ELONG", (double)elongation[k]);
      liFeature->SetField("CIRC", (double)circularity[k]);
      liFeature->SetField("RECT", (double)rectangularity[k]);
      liFeature->SetField("COMPAC", (double)compactness[k]);
      liFeature->SetField("CLASS", "non-classified");
    }

    // initiate polygon start
    OGRLinearRing initialRing;
    initialRing.setCoordinateDimension(2);
    initialRing.addPoint(oX + (double)linelists[k][0].sX * mX, oY + mY * (double)linelists[k][0].sY);
    initialRing.addPoint(oX + (double)linelists[k][0].eX * mX, oY + mY * (double)linelists[k][0].eY);
    linelists[k].erase(linelists[k].begin());

    // construct polygon from lines
    while (linelists[k].size() > 0 && multiring == 0)
    {
      vector<LINE>::iterator it = linelists[k].begin();
      for (; it != linelists[k].end(); ++it)
      {
        double ltX = initialRing.getX(initialRing.getNumPoints() - 1);
        double ltY = initialRing.getY(initialRing.getNumPoints() - 1);
        double csX = oX + (double)it->sX * mX;
        double csY = oY + mY * (double)it->sY;
        double ceX = oX + (double)it->eX * mX;
        double ceY = oY + mY * (double)it->eY;

        if ((csX == ltX) && (csY == ltY))
        {
          initialRing.addPoint(ceX, ceY);
          linelists[k].erase(it);
          break;
        }
        if ((ceX == ltX) && (ceY == ltY))
        {
          initialRing.addPoint(csX, csY);
          linelists[k].erase(it);
          break;
        }
        if (it == linelists[k].end() - 1)
        {
          multiring = 1;
          break;
        }
      }
    }
    initialRing.closeRings();

    // simplify poligons
    // remove colinear vertices
    OGRLinearRing rings;
    float pointPrevX = 0, pointPrevY = 0;
    for (int i = 0; i < initialRing.getNumPoints(); i++)
    {
      OGRPoint point;
      initialRing.getPoint(i, &point);

      // start
      if (i == 0)
      {
        rings.addPoint(&point);
        pointPrevX = point.getX();
        pointPrevY = point.getY();
        continue;
      }
      // end vertex
      if (i == initialRing.getNumPoints() - 1)
      {
        rings.addPoint(&point);
        continue;
      }

      OGRPoint pointNext;
      initialRing.getPoint(i + 1, &pointNext);
      //     | x1 y1 1 |
      // det | x2 y2 1 | = 0 => p1,p2,p3 are colinear
      //     | x3 y3 1 |
      // x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2) == 0
      // only if not colinear with previous and next
      if (pointPrevX * (point.getY() - pointNext.getY()) +
              point.getX() * (pointNext.getY() - pointPrevY) +
              pointNext.getX() * (pointPrevY - point.getY()) !=
          0)
      {
        rings.addPoint(&point);
        pointPrevX = point.getX();
        pointPrevY = point.getY();
      }
    }

    OGRPolygon polygon;
    polygon.addRing(&rings);
    liFeature->SetFID(k);
    
    // if (multiring == 1)
    // {      
    //   OGRFeature *auxFeature = liLayer->GetFeature(k-1);      
    //   OGRPolygon *auxPolygon = (OGRPolygon *) auxFeature->GetGeometryRef();         
    //   auxPolygon->addRing(&rings);
    //   liFeature->SetGeometry(auxPolygon);      

    //   if (liLayer->SetFeature(liFeature) != OGRERR_NONE)
    //   {
    //      printf("\nERROR: Failed to set feature in vector layer.\n");
    //      exit(1);
    //   }
    //   liLayer->SyncToDisk();
    //   //OGRFeature::DestroyFeature(liFeature);
    // }    
    // else
    // {        
      liFeature->SetGeometry(&polygon);   
      if (liLayer->CreateFeature(liFeature) != OGRERR_NONE)
      {
        printf("\nERROR: Failed to create feature in vector layer.\n");
        exit(1);
      }      
      liLayer->SyncToDisk();
      OGRFeature::DestroyFeature(liFeature);
    //}

    //printf("\nk: %lu, nFeature: %d\n\n", k, liLayer->GetFeatureCount(true));
    // printf("%d\n", liLayer->GetFeatureCount(true));

    GDALTermProgress((float)(k + 1) / (float)(m_labels), NULL, NULL);
  }
  GDALTermProgress(1.0f, NULL, NULL);

  #if GDALVER >= 2
  GDALClose(liDS);
  #else
  OGRDataSource::DestroyDataSource(liDS);
  #endif
}
