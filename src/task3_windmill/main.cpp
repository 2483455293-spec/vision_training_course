#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/stat.h>
#include <vector>
using namespace cv;
using namespace std;

bool mkdirIfNotExist(const string &path) {
  struct stat st;
  if (stat(path.c_str(), &st) == 0)
    return true;
  return mkdir(path.c_str(), 0755) == 0;
}

double calcAngle(Point2f p, Point2f center) {
  Point2f d = p - center;
  double rad = atan2(d.y, d.x);
  double deg = rad * 180.0 / CV_PI;
  return deg < 0 ? deg + 360.0 : deg;
}

double angleDiff(double a1, double a2) {
  double d = fabs(a1 - a2);
  return d > 180.0 ? 360.0 - d : d;
}

struct RingFeature {
  Point2f center;
  float radius;
  int childCount;
};

struct RBlobFeature {
  Point2f center;
  double area;
};

struct LeafTrack {
  int trackId;
  Point2f lastCenter;
  float smoothRadius;
  double angle;
  int lostFrameCnt;
  bool active;
  LeafTrack(int id_, Point2f c, float r, double ang)
      : trackId(id_), lastCenter(c), smoothRadius(r), angle(ang),
        lostFrameCnt(0), active(true) {}
};

void parseMaskContours(const Mat &mask, vector<RingFeature> &ringList,
                       vector<RBlobFeature> &rBlobList) {
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  vector<pair<vector<Point>, Vec4i>> topLevel;
  for (size_t i = 0; i < contours.size(); i++) {
    if (hierarchy[i][3] != -1)
      continue;
    topLevel.emplace_back(contours[i], hierarchy[i]);
  }
  for (auto &item : topLevel) {
    auto &cnt = item.first;
    auto &hierInfo = item.second;
    double areaVal = contourArea(cnt);
    if (areaVal < 200.0)
      continue;
    double periVal = arcLength(cnt, true);
    if (periVal < 1e-6)
      continue;
    double circularity = 4.0 * CV_PI * areaVal / (periVal * periVal);
    Point2f circC;
    float circR;
    minEnclosingCircle(cnt, circC, circR);
    int subCnt = 0;
    int childIdx = hierInfo[2];
    for (int c = childIdx; c != -1; c = hierarchy[c][0])
      subCnt++;
    if (circularity >= 0.80) {
      ringList.push_back({circC, circR, subCnt});
    } else {
      if (areaVal >= 200.0 && areaVal <= 400.0) {
        rBlobList.push_back({circC, areaVal});
      }
    }
  }
}

bool selectRMarker(const vector<RingFeature> &ringList,
                   const vector<RBlobFeature> &rBlobList,
                   bool prevRValid, Point2f prevR,
                   Point2f &outRCenter)
{
  Point2f hub{0, 0};
  int validRingNum = 0;
  for (const auto &rf : ringList)
  {
    if (rf.childCount >= 1)
    {
      hub += rf.center;
      validRingNum++;
    }
  }
  if (validRingNum == 0 || rBlobList.empty())
    return false;

  hub.x /= validRingNum;
  hub.y /= validRingNum;

  const double R_BLOB_MIN_AREA = 180.0;
  const double MAX_BLOB_TO_HUB = 180.0;
  bool ringClusterEnough = (validRingNum >= 2);

  vector<RBlobFeature> validBlobCandidates;
  for (const auto &b : rBlobList)
  {
    if (b.area < R_BLOB_MIN_AREA)
      continue;
    double dHub = sqrt(pow(b.center.x - hub.x, 2) + pow(b.center.y - hub.y, 2));
    if(ringClusterEnough && (dHub > MAX_BLOB_TO_HUB))
      continue;
    validBlobCandidates.push_back(b);
  }
  if (validBlobCandidates.empty())
    return false;

  int bestIdx = -1;
  double bestDist = 1e9;
  if (prevRValid)
  {
    for (size_t k = 0; k < validBlobCandidates.size(); k++)
    {
      double d = sqrt(pow(validBlobCandidates[k].center.x - prevR.x, 2)
                     + pow(validBlobCandidates[k].center.y - prevR.y, 2));
      if (d < bestDist)
      {
        bestDist = d;
        bestIdx = static_cast<int>(k);
      }
    }
  }
  else
  {
    for (size_t k = 0; k < validBlobCandidates.size(); k++)
    {
      double d = sqrt(pow(validBlobCandidates[k].center.x - hub.x, 2)
                     + pow(validBlobCandidates[k].center.y - hub.y, 2));
      if (d < bestDist)
      {
        bestDist = d;
        bestIdx = static_cast<int>(k);
      }
    }
  }
  outRCenter = validBlobCandidates[bestIdx].center;
  return true;
}

void updateLeafTracks(vector<LeafTrack> &trackVec,
                      const vector<RingFeature> &validLeafObserves,
                      Point2f rCenter, bool rValid, int &nextAssignId,
                      float distThresh, double angThresh, int maxLostThresh) {
  (void)distThresh;
  for (auto &tr : trackVec) {
    if (tr.active)
      tr.lostFrameCnt++;
  }
  vector<bool> obsMatched(validLeafObserves.size(), false);
  for (auto &tr : trackVec) {
    if (!tr.active)
      continue;
    double bestAngDiff = 1e12;
    int bestObs = -1;
    for (size_t o = 0; o < validLeafObserves.size(); o++) {
      if (obsMatched[o])
        continue;
      const auto &obs = validLeafObserves[o];
      if (!rValid)
        continue;
      double obsAng = calcAngle(obs.center, rCenter);
      double dAng = angleDiff(tr.angle, obsAng);
      if (dAng > angThresh)
        continue;
      if (dAng < bestAngDiff) {
        bestAngDiff = dAng;
        bestObs = static_cast<int>(o);
      }
    }
    if (bestObs != -1) {
      auto &obs = validLeafObserves[bestObs];
      tr.lastCenter = obs.center;
      tr.smoothRadius = 0.5f * tr.smoothRadius + 0.5f * obs.radius;
      tr.angle = calcAngle(obs.center, rCenter);
      tr.lostFrameCnt = 0;
      obsMatched[bestObs] = true;
    }
  }
  for (size_t o = 0; o < validLeafObserves.size(); o++) {
    if (!obsMatched[o]) {
      auto &obs = validLeafObserves[o];
      double ang = 0;
      if (rValid)
        ang = calcAngle(obs.center, rCenter);
      trackVec.emplace_back(nextAssignId++, obs.center, obs.radius, ang);
    }
  }
  for (auto &tr : trackVec) {
    if (tr.active && tr.lostFrameCnt > maxLostThresh) {
      tr.active = false;
    }
  }
}

bool processVideo(const string &srcPath, const string &outDir) {
  if (!mkdirIfNotExist(outDir)) {
    cerr << "无法创建输出目录:" << outDir << endl;
    return false;
  }
  VideoCapture cap(srcPath);
  if (!cap.isOpened()) {
    cerr << "打开视频失败:" << srcPath << endl;
    return false;
  }
  int w = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
  int h = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
  double fps = cap.get(CAP_PROP_FPS);
  string overlayPath = outDir + "/recognition_overlay.mp4";
  string binaryPath = outDir + "/binary_process.mp4";
  VideoWriter writerOverlay(
      overlayPath, VideoWriter::fourcc('m', 'p', '4', 'v'), fps, Size(w, h));
  VideoWriter writerBinary(binaryPath, VideoWriter::fourcc('m', 'p', '4', 'v'),
                          fps, Size(w, h), false);
  if (!writerOverlay.isOpened() || !writerBinary.isOpened()) {
    cerr << "VideoWriter打开失败\n";
    cap.release();
    return false;
  }
  vector<LeafTrack> trackList;
  int nextId = 0;
  int frameCounter = 0;
  const float MATCH_DIST = 160.0f;
  const double ANGLE_THRESH = 50;
  const int MAX_LOST_FRAME = 30;
  const float MAX_R_LEAF_DIST = 330.0f;

  bool prevRValid = false;
  Point2f prevRPoint;
  int rLostCnt = 0;
  const int R_MAX_LOST = 40;


  Mat frame;
  while (cap.read(frame)) {
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    Mat maskRedLow, maskRedHigh, maskTotal;
    inRange(hsv, Scalar(0, 45, 50), Scalar(12, 255, 255), maskRedLow);
    inRange(hsv, Scalar(168, 45, 50), Scalar(179, 255, 255), maskRedHigh);
    bitwise_or(maskRedLow, maskRedHigh, maskTotal);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    morphologyEx(maskTotal, maskTotal, MORPH_CLOSE, kernel);
    vector<RingFeature> ringFeats;
    vector<RBlobFeature> rBlobFeats;
    parseMaskContours(maskTotal, ringFeats, rBlobFeats);

    Point2f rCenter;
    bool curRIsValid = selectRMarker(ringFeats, rBlobFeats, prevRValid, prevRPoint, rCenter);
    bool rIsValid;
    if (curRIsValid)
    {
      rIsValid = true;
      prevRValid = true;
      prevRPoint = rCenter;
      rLostCnt = 0;
    }
    else
    {
      rLostCnt++;
      if(rLostCnt > R_MAX_LOST)
      {
        prevRValid = false;
      }
      rIsValid = false;
    }

    vector<RingFeature> validLeafObs;
    for (auto &rf : ringFeats) {
      if (rf.childCount >= 2) {
        bool ok = true;
        if (rIsValid) {
          Point2f d = rf.center - rCenter;
          double dist = sqrt(d.x * d.x + d.y * d.y);
          if (dist > MAX_R_LEAF_DIST)
            ok = false;
        }
        if (ok)
          validLeafObs.push_back(rf);
      }
    }
    updateLeafTracks(trackList, validLeafObs, rCenter, rIsValid, nextId,
                     MATCH_DIST, ANGLE_THRESH, MAX_LOST_FRAME);
    Mat drawCanvas = frame.clone();
    if (rIsValid) {
      circle(drawCanvas, rCenter, 6, Scalar(0, 255, 0), -1);
      line(drawCanvas, rCenter - Point2f(18, 0), rCenter + Point2f(18, 0),
           Scalar(0, 255, 0), 2);
      line(drawCanvas, rCenter - Point2f(0, 18), rCenter + Point2f(0, 18),
           Scalar(0, 255, 0), 2);
    }
    int textY = 25;
    for (auto &tr : trackList) {
      if (!tr.active)
        continue;
      if (rIsValid) {
        Point2f dTr = tr.lastCenter - rCenter;
        double distTr = sqrt(dTr.x * dTr.x + dTr.y * dTr.y);
        if (distTr > MAX_R_LEAF_DIST) {
          tr.active = false;
          continue;
        }
      }
      if (tr.lostFrameCnt == 0) {
        Scalar col = Scalar(0, 255, 0);
        circle(drawCanvas, tr.lastCenter, static_cast<int>(tr.smoothRadius),
               col, 2);
        circle(drawCanvas, tr.lastCenter, 4, Scalar(0, 255, 255), -1);
        if (rIsValid) {
          line(drawCanvas, tr.lastCenter, rCenter, Scalar(255, 0, 0), 2);
        }
        putText(drawCanvas, to_string(tr.trackId),
                Point(static_cast<int>(tr.lastCenter.x) + 10,
                      static_cast<int>(tr.lastCenter.y) - 10),
                FONT_HERSHEY_SIMPLEX, 0.8, col, 2);
        String info = format("ID:%d DETECTED", tr.trackId);
        putText(drawCanvas, info, Point(12, textY), FONT_HERSHEY_SIMPLEX, 0.6,
                col, 2);
        textY += 24;
      } else {
        Scalar col = Scalar(0, 0, 255);
        String info = format("ID:%d LOST", tr.trackId);
        putText(drawCanvas, info, Point(12, textY), FONT_HERSHEY_SIMPLEX, 0.6,
                col, 2);
        textY += 24;
      }
    }
    writerOverlay.write(drawCanvas);
    writerBinary.write(maskTotal);
    frameCounter++;
  }
  cap.release();
  writerOverlay.release();
  writerBinary.release();
  cout << "完成输出:" << overlayPath << endl;
  return true;
}

int main() {
  string t3_src = "resources/task_3.mp4";
  string t3_out = "result/task3_windmill/task_3";
  string t4_src = "resources/task_4.mp4";
  string t4_out = "result/task3_windmill/task_4";
  cout << "===== 处理task_3.mp4 =====\n";
  processVideo(t3_src, t3_out);
  cout << "===== 处理task_4.mp4 =====\n";
  processVideo(t4_src, t4_out);
  cout << "全部结束\n";
  return 0;
}
