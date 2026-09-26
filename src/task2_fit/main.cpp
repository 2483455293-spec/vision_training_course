#include <Eigen/Dense>
#include <algorithm>
#include <ceres/ceres.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
using namespace cv;
using namespace std;
const double PI = acos(-1.0);
double normalize_angle(double ang) {
  ang = fmod(ang, 2 * PI);
  if (ang > PI)
    ang -= 2 * PI;
  if (ang < -PI)
    ang += 2 * PI;
  return ang;
}
double unwrap_angle(double wrapped_ang, double prev_unwrapped) {
  double diff = normalize_angle(wrapped_ang - prev_unwrapped);
  return prev_unwrapped + diff;
}
double omega_calculate(double deltat, double theta_x, double theta_xplus2) {
  double diff = theta_xplus2 - theta_x;
  double omega = diff / deltat;
  return omega;
}


//=====绘图=======
static const int PLOT_L = 70, PLOT_R = 70, PLOT_T = 50, PLOT_B = 50;
inline Point toPix(double x, double y, const Size &sz, double xmin, double xmax,
                   double ymin, double ymax) {
  double sx = (sz.width - PLOT_L - PLOT_R) / (xmax - xmin);
  double sy = (sz.height - PLOT_T - PLOT_B) / (ymax - ymin);
  return Point(PLOT_L + cvRound((x - xmin) * sx),
               sz.height - PLOT_B - cvRound((y - ymin) * sy));
}
void drawCurve(const vector<double> &x, const vector<double> &y, Mat &canvas,
               Scalar color, double xmin, double xmax, double ymin, double ymax,
               int thickness = 2) {
  Size sz = canvas.size();
  for (size_t i = 1; i < x.size(); ++i)
    line(canvas, toPix(x[i - 1], y[i - 1], sz, xmin, xmax, ymin, ymax),
         toPix(x[i], y[i], sz, xmin, xmax, ymin, ymax), color, thickness,
         LINE_AA);
}
struct SineResidual {
  SineResidual(double t, double omega) : x_(t), y_(omega) {}
  template <typename T> bool operator()(const T *const p, T *residual) const {
    residual[0] = T(y_) - (p[0] + p[1] * ceres::sin(p[2] * T(x_) + p[3]));
    return true;
  }
  double x_, y_;
};
int main() {
  VideoCapture cap("resources/task_2.mp4");
  if (!cap.isOpened()) {
    cerr << "Error opening video stream or file" << std::endl;
    return -1;
  }
  int width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
  int height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
  double fps = cap.get(CAP_PROP_FPS);
  VideoWriter writer("result/task2_fit/tracking_overlay.mp4",
                     VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                     Size(width, height));
  Mat frame;
  vector<double> t_list;
  vector<double> theta_unwrapped_list;
  vector<double> omega_list;
  int frame_count = 0;
  double last_unwrapped = 0.0;
  const double min_contour_area = 20.0;


  //====================任务内容1：识别青色目标，提取中心坐标，计算包裹角度、展开角度====================
  while (true) {
    bool ret = cap.read(frame);
    if (!ret) {
      break;
    }
    vector<std::vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat hsv, target;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(80, 120, 100), Scalar(150, 255, 255), target);
    morphologyEx(target, target, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(5, 5)));
    findContours(target, contours, hierarchy, RETR_EXTERNAL,
                 CHAIN_APPROX_SIMPLE);
    Point2f targetcenter;
    const Point2f center(480, 360);
    float radius = 0;
    double theta_unwrapped = 0.0;
    double omega = 0.0;
    int best_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      double area = contourArea(contours[i]);
      if (area > max_area) {
        max_area = area;
        best_idx = static_cast<int>(i);
      }
    }
    if (best_idx != -1 && max_area > min_contour_area) {
      minEnclosingCircle(contours[best_idx], targetcenter, radius);
      circle(frame, targetcenter, cvRound(radius), Scalar(0, 255, 255), 2);
      circle(frame, targetcenter, 3, Scalar(0, 255, 255), -1);
      double theta_wrapped =
          atan2(center.y - targetcenter.y, targetcenter.x - center.x);
      if (frame_count == 0) {
        theta_unwrapped = theta_wrapped;
      } else {
        theta_unwrapped = unwrap_angle(theta_wrapped, last_unwrapped);
      }
      last_unwrapped = theta_unwrapped;
      if (frame_count != 0) {
        omega = omega_calculate(1.0 / fps, theta_unwrapped_list.back(),
                                theta_unwrapped);
      }
    }
    t_list.push_back(frame_count / fps);
    theta_unwrapped_list.push_back(theta_unwrapped);
    omega_list.push_back(omega);
    ostringstream oss;
    oss << "theta_unwrap=" << fixed << setprecision(2) << theta_unwrapped
        << " ";
    oss << "omega=" << fixed << setprecision(2) << omega;
    if (best_idx != -1 && max_area > min_contour_area) {
      putText(frame, oss.str(), Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.75,
              Scalar(0, 255, 255), 2);
    }
    writer.write(frame);
    frame_count++;
  }
  const int kSample = 5;
  vector<double> t_fit, omega_fit;
  for (size_t i = kSample; i < theta_unwrapped_list.size(); i += kSample) {
    double omega =
        omega_calculate(kSample / fps, theta_unwrapped_list[i - kSample],
                        theta_unwrapped_list[i]);
    t_fit.push_back(t_list[i]);
    omega_fit.push_back(omega);
  }


  //====================任务内容2：Ceres估计模型参数（拟合角速度路线）====================
  double p[4] = {1.2, 0.5, 1.5, 0.3};
  ceres::Problem problem;
  for (size_t i = 0; i < t_fit.size(); ++i) {
    auto *cost = new ceres::AutoDiffCostFunction<SineResidual, 1, 4>(
        new SineResidual(t_fit[i], omega_fit[i]));
    problem.AddResidualBlock(cost, nullptr, p);
  }
  problem.SetParameterLowerBound(p, 1, 0.0);
  problem.SetParameterLowerBound(p, 2, 0.0);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";
  p[3] = normalize_angle(p[3]);
  cout << "Estimated parameters: b=" << p[0] << ", A=" << p[1] << ", Ω=" << p[2]
       << ", φ=" << p[3] << "\n";
  vector<double> t_raw(t_list.begin() + 1, t_list.end());
  vector<double> omega_raw(omega_list.begin() + 1, omega_list.end());
  vector<double> t_curve, omega_curve;
  const int kCurvePts = 600;
  for (int i = 0; i < kCurvePts; ++i) {
    double t =
        t_raw.front() + (t_raw.back() - t_raw.front()) * i / (kCurvePts - 1);
    t_curve.push_back(t);
    omega_curve.push_back(p[0] + p[1] * sin(p[2] * t + p[3]));
  }


  //====================任务内容3：计算残差RMSE，绘制观测‑拟合对比图、残差图、角速度曲线====================
  vector<double> r_list;
  for (size_t i = 0; i < t_fit.size(); ++i)
    r_list.push_back(omega_fit[i] -
                     (p[0] + p[1] * sin(p[2] * t_fit[i] + p[3])));
  double rmse = 0.0;
  for (double r : r_list)
    rmse += r * r;
  rmse = sqrt(rmse / r_list.size());
  double xmin = *min_element(t_raw.begin(), t_raw.end());
  double xmax = *max_element(t_raw.begin(), t_raw.end());
  double ymin = *min_element(omega_raw.begin(), omega_raw.end());
  double ymax = *max_element(omega_raw.begin(), omega_raw.end());
  double dx = (xmax - xmin) * 0.05, dy = (ymax - ymin) * 0.05;
  xmin -= dx;
  xmax += dx;
  ymin -= dy;
  ymax += dy;

  Mat fit_comparison(1000, 1500, CV_8UC3, Scalar(0, 0, 0));
  Mat residuals = fit_comparison.clone();
  Mat angular_velocity = fit_comparison.clone();

  drawCurve(t_raw, omega_raw, fit_comparison, Scalar(255, 255, 255), xmin, xmax,
            ymin, ymax, 1);
  drawCurve(t_curve, omega_curve, fit_comparison, Scalar(0, 255, 255), xmin,
            xmax, ymin, ymax, 2);
  putText(fit_comparison, "fit_comparison", Point(PLOT_L, 30),
          FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2, LINE_8);
  const char *legend_names[2] = {"original sample", "Ceres fit"};
  Scalar legend_colors[2] = {Scalar(255, 255, 255), Scalar(0, 255, 255)};
  for (int i = 0; i < 2; ++i) {
    int ly = 55 + i * 26;
    line(fit_comparison, Point(1130, ly), Point(1170, ly), legend_colors[i], 3,
         LINE_8);
    putText(fit_comparison, legend_names[i], Point(1180, ly + 5),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_8);
  }

  double r_abs = 0.0;
  for (double r : r_list)
    r_abs = max(r_abs, fabs(r));
  double rmin = -1.2 * r_abs, rmax = 1.2 * r_abs;
  line(residuals, toPix(xmin, 0.0, residuals.size(), xmin, xmax, rmin, rmax),
       toPix(xmax, 0.0, residuals.size(), xmin, xmax, rmin, rmax),
       Scalar(255, 255, 255), 1, LINE_8);
  drawCurve(t_fit, r_list, residuals, Scalar(255, 255, 255), xmin, xmax, rmin,
            rmax, 1);
  putText(residuals, "residual", Point(PLOT_L, 30), FONT_HERSHEY_SIMPLEX, 0.7,
          Scalar(255, 255, 255), 2, LINE_8);
  line(residuals, Point(1130, 55), Point(1170, 55), Scalar(255, 255, 255), 3,
       LINE_8);
  putText(residuals, "residual", Point(1180, 60), FONT_HERSHEY_SIMPLEX, 0.5,
          Scalar(255, 255, 255), 1, LINE_8);
  cout << fixed << setprecision(6) << "RMSE = " << rmse << "\n";

  drawCurve(t_curve, omega_curve, angular_velocity, Scalar(0, 255, 255), xmin,
            xmax, ymin, ymax, 2);
  putText(angular_velocity, "angular_velocity", Point(PLOT_L, 30),
          FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2, LINE_8);
  int ly = 55;
  line(angular_velocity, Point(1130, ly), Point(1170, ly), Scalar(0, 255, 255),
       3, LINE_8);
  putText(angular_velocity, "angular_velocity", Point(1180, ly + 5),
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_8);

  imwrite("result/task2_fit/fit_comparison.png", fit_comparison);
  imwrite("result/task2_fit/angular_velocity.png", angular_velocity);
  imwrite("result/task2_fit/residuals.png", residuals);


  //====================任务内容4：输出md报告、带标记视频====================
  ofstream md("result/task2_fit/task2_fit_result.md");
  md << "# Task2 合成旋转视频参数拟合报告\n";
  md << "## 任务1：识别青色目标\n";
  md << "- HSV颜色分割 + "
        "形态学开运算，选取最大面积轮廓，minEnclosingCircle获取小球中心\n";
  md << "- "
        "atan2得到原始包裹角度，执行unwrap角度解缠绕，内存保存展开后的连续角度"
        "序列\n";
  md << "- 使用间隔kSample帧差分（基于展开角度序列）计算观测角速度\n\n";
  md << "## 任务2：参数估计\n";
  md << "- 模型 $\\omega(t)=b+A\\sin(\\Omega "
        "t+\\varphi)$，直接拟合角速度，不需要估计初始角度$\\theta_0$\n";
  md << "- 求解：Ceres AutoDiff自动求导，DENSE_QR\n";
  md << "- 初值：b=" << p[0] << ", A=" << p[1] << ", Ω=" << p[2]
     << ", φ=" << p[3] << "\n";
  md << "- 参数约束：A>0, Ω>0；相位φ输出归一化至 (-π, π)\n";
  md << "- Ceres求解状态：" << (summary.IsSolutionUsable() ? "可用" : "不可用")
     << "\n\n";
  md << "|参数|数值|单位|\n|---|---|---|\n";
  md << "|b(平均角速度)|" << p[0] << "|rad/s|\n";
  md << "|A(振幅)|" << p[1] << "|rad/s|\n";
  md << "|Ω(频率参数)|" << p[2] << "|rad/s|\n";
  md << "|φ(相位)|" << p[3] << "|rad|\n\n";
  md << "## 任务3误差\n";
  md << "- 角速度RMSE：" << rmse << " rad/s\n";
  md << "- 有效拟合样本：" << t_fit.size() << "\n";
  md << "- 帧采样间隔：kSample=" << kSample << "帧\n";
  md << "- 参与拟合帧范围：第" << kSample << "帧 ~ 第" << (frame_count - 1)
     << "帧\n\n";
  md << "## 任务4输出\n";
  md << "- tracking_overlay.mp4 带标记视频\n";
  md << "- fit_comparison.png "
        "观测‑拟合对比（黑色背景；原始观测白色，拟合曲线黄色）\n";
  md << "- residuals.png 残差图（黑色背景；残差曲线白色）\n";
  md << "- angular_velocity.png 角速度曲线（黑色背景；曲线黄色）\n";
  md.close();
  writer.release();
  cap.release();
  return 0;
}
