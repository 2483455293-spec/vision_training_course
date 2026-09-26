#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main() {
  // ========1.读图与颜色转换======== //

  // 检查图像读取
  Mat img = imread("resources/test_image.jpg");
  if (img.empty()) {
    cerr << "Cannot read image!\n";
    return 1;
  }

  // 灰度图
  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  imwrite("result/task1_images/gray.png", gray);

  // HSV图
  Mat hsv;
  cvtColor(img, hsv, COLOR_BGR2HSV);
  Mat channels[3];
  split(hsv, channels);
  imwrite("result/task1_images/hsv_h.png", channels[0]);
  imwrite("result/task1_images/hsv_s.png", channels[1]);
  imwrite("result/task1_images/hsv_v.png", channels[2]);

  // ========2.滤波对比======== //
  Mat meanImg, gaussianImg, medianImg;
  blur(img, meanImg, Size(5, 5));
  GaussianBlur(img, gaussianImg, Size(5, 5), 1.5);
  medianBlur(img, medianImg, 5);

  imwrite("result/task1_images/mean_filter.png", meanImg);
  imwrite("result/task1_images/gaussian_filter.png", gaussianImg);
  imwrite("result/task1_images/median_filter.png", medianImg);


  // =========3.红色提取=========//
  Mat maskLow, maskHigh, red_mask;
  cvtColor(img, hsv, COLOR_BGR2HSV);
  inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), maskLow);
  inRange(hsv, Scalar(170, 100, 100), Scalar(179, 255, 255), maskHigh);
  bitwise_or(maskLow, maskHigh, red_mask);

  imwrite("result/task1_images/red_mask.png", red_mask);


  // =========4.形态学与轮廓========= //

  // 腐蚀、膨胀、开运算、闭运算
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat dilated, eroded, opened, closed;

  erode(red_mask, eroded, kernel);
  dilate(red_mask, dilated, kernel);
  morphologyEx(red_mask, opened, MORPH_OPEN, kernel);
  morphologyEx(red_mask, closed, MORPH_CLOSE, kernel);

  imwrite("result/task1_images/erode.png", eroded);
  imwrite("result/task1_images/dilate.png", dilated);
  imwrite("result/task1_images/open.png", opened);
  imwrite("result/task1_images/close.png", closed);

  // 提取外层轮廓和面积筛选绘制
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(closed, contours, hierarchy, RETR_EXTERNAL,
               CHAIN_APPROX_SIMPLE); // 使用闭运算结果作为轮廓输入

  Mat result = img.clone();
  const double area_thresh = 300.0;

  cout << "\n-----筛选后轮廓面积-----\n";
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = contourArea(contours[i]);
    if (area < area_thresh)
      continue;

    Rect box = boundingRect(contours[i]);
    drawContours(result, contours, static_cast<int>(i), Scalar(0, 255, 0), 2);
    rectangle(result, box, Scalar(0, 0, 255), 2);

    // 在图上打印轮廓面积
    string areaText = to_string(static_cast<int>(area));
    putText(result, areaText, Point(box.x, box.y - 6), FONT_HERSHEY_SIMPLEX,
            0.5, Scalar(255, 0, 0), 1);
    cout << areaText << endl;
  }
  imwrite("result/task1_images/contours_boxes.png", result);

  
  // =========5.绘制与变换 ========= //

  // 绘制圆、矩形和⽂字
  Mat draw_img = img.clone();
  circle(draw_img, Point(220, 220), 90, Scalar(255, 0, 0), 2);
  rectangle(draw_img, Point(60, 60), Point(260, 260), Scalar(0, 0, 255), 2);
  putText(draw_img, "Tulip Task1 Demo", Point(320, 110), FONT_HERSHEY_SIMPLEX,
          1.0, Scalar(0, 255, 0), 2);
  imwrite("result/task1_images/drawing.png", draw_img);

  // 绕图像中心旋转35°
  Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);
  Mat rot_mat = getRotationMatrix2D(img_center, 35.0, 1.0);
  Mat rotated_img;
  warpAffine(img, rotated_img, rot_mat, img.size());
  imwrite("result/task1_images/rotated_35deg.png", rotated_img);

  // 裁剪左上角1/4，宽、高各取一半
  Mat crop_top_left = img(Rect(0, 0, img.cols / 2, img.rows / 2));
  imwrite("result/task1_images/crop_top_left.png", crop_top_left);

  cout << "Task1全部图片已经输出!\n";
  return 0;
}