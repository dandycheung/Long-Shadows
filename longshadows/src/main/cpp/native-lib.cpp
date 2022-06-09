#include <jni.h>
#include <string>
#include <vector>
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <malloc.h>
#include <android/log.h>

#define PI 3.1415926535897932384
#define CORRECTIVE_OFFSET 3
#define ALPHA_0 0
#define REFERENCE_POINT_OFFSET 2000

using namespace std;

struct ShadowPath {
    vector<pair<int, int> > points;
    pair<int, int> startPointOne;
    pair<int, int> startPointTwo;
    pair<int, int> endPointOne;
    pair<int, int> endPointTwo;
};

vector<vector<pair<int, int> > >
contours(JNIEnv *env, const int arr[], int width, int height, int backgroundColor) {
//    int mat[height + 1][width + 1];
//    int id[height + 1][width + 1];

    int r = height + 1;
    int c = width + 1;

    int **mat = (int **) malloc(r * sizeof(int *));
    for (int i = 0; i < r; i++)
        mat[i] = (int *) malloc(c * sizeof(int));

    int **id = (int **) malloc(r * sizeof(int *));
    for (int i = 0; i < r; i++)
        id[i] = (int *) malloc(c * sizeof(int));

    queue<pair<int, int> > q;

    vector<vector<pair<int, int> > > ans;
    ans.clear();

    vector<pair<int, int> > temp;

    int n = width * height;

    for (int i = 0; i < n; i++) {
        mat[i / width][i % width] = arr[i];
        id[i / width][i % width] = 0;
    }

    int cnt = 0, x, y, x1, y1;
    pair<int, int> tp;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (id[i][j] == 0 && mat[i][j] != backgroundColor && mat[i][j] != ALPHA_0) {
                cnt++;
                temp.clear();

                while (!q.empty())
                    q.pop();

                q.push(make_pair(i, j));

                while (!q.empty()) {
                    tp = q.front();
                    q.pop();
                    x = tp.first;
                    y = tp.second;

                    if (x < 0 || y < 0 || x >= height || y >= width)
                        continue;

                    if (mat[x][y] == backgroundColor || mat[x][y] == ALPHA_0)
                        continue;

                    if (id[x][y] != 0) {
                        assert(id[x][y] == cnt);
                        continue;
                    }

                    assert(id[x][y] == 0);
                    assert(mat[x][y] != backgroundColor && mat[x][y] != ALPHA_0);

                    id[x][y] = cnt;

                    bool flag = false;

                    for (int i1 = -1; i1 < 2; i1++) {
                        for (int j1 = -1; j1 < 2; j1++) {
                            x1 = x + i1;
                            y1 = y + j1;

                            if (x1 < 0 || y1 < 0 || x1 >= height || y1 >= width) {
                                flag = true;
                                break;
                            }

                            if (mat[x1][y1] == backgroundColor || mat[x1][y1] == ALPHA_0) {
                                flag = true;
                                break;
                            }
                        }

                        if (flag)
                            break;
                    }

                    if (flag)
                        temp.emplace_back(y, x);

                    q.push(make_pair(x - 1, y - 1));
                    q.push(make_pair(x - 1, y));
                    q.push(make_pair(x - 1, y + 1));
                    q.push(make_pair(x, y - 1));
                    q.push(make_pair(x, y + 1));
                    q.push(make_pair(x + 1, y - 1));
                    q.push(make_pair(x + 1, y));
                    q.push(make_pair(x + 1, y + 1));
                }

                ans.push_back(temp);
            }
        }
    }

    free(mat);
    free(id);

    return ans;
}

jobjectArray convertToObjectArray(JNIEnv *env, vector<ShadowPath> shadowPaths) {
    jclass shadowPathClass = (env)->FindClass("com/sdsmdg/harjot/longshadows/models/ShadowPath");
    jobjectArray shadowObjectArray = (env)->NewObjectArray(shadowPaths.size(), shadowPathClass,
                                                           nullptr);
    jmethodID shadowClassConstructor = (env)->GetMethodID(shadowPathClass, "<init>",
                                                          "([Lcom/sdsmdg/harjot/longshadows/models/Point2D;"
                                                                  "Lcom/sdsmdg/harjot/longshadows/models/Point2D;"
                                                                  "Lcom/sdsmdg/harjot/longshadows/models/Point2D;"
                                                                  "Lcom/sdsmdg/harjot/longshadows/models/Point2D;"
                                                                  "Lcom/sdsmdg/harjot/longshadows/models/Point2D;)"
                                                                  "V");
    jobject shadowPathObject;

    for (int i = 0; i < shadowPaths.size(); i++) {
        vector<pair<int, int> > points = shadowPaths[i].points;

        jclass pointClass = (env)->FindClass("com/sdsmdg/harjot/longshadows/models/Point2D");
        jobjectArray pointArray = (env)->NewObjectArray(points.size(), pointClass, nullptr);
        jmethodID pointClassConstructor = (env)->GetMethodID(pointClass, "<init>", "(II)V");
        jobject pointObject;

        for (int j = 0; j < points.size(); j++) {
            pointObject = (env)->NewObject(pointClass, pointClassConstructor, points[j].first,
                                           points[j].second);
            (env)->SetObjectArrayElement(pointArray, j, pointObject);
            (env)->DeleteLocalRef(pointObject);
        }

        jobject startPointOne = (env)->NewObject(pointClass, pointClassConstructor,
                                                 shadowPaths[i].startPointOne.first,
                                                 shadowPaths[i].startPointOne.second);
        jobject startPointTwo = (env)->NewObject(pointClass, pointClassConstructor,
                                                 shadowPaths[i].startPointTwo.first,
                                                 shadowPaths[i].startPointTwo.second);
        jobject endPointOne = (env)->NewObject(pointClass, pointClassConstructor,
                                               shadowPaths[i].endPointOne.first,
                                               shadowPaths[i].endPointOne.second);
        jobject endPointTwo = (env)->NewObject(pointClass, pointClassConstructor,
                                               shadowPaths[i].endPointTwo.first,
                                               shadowPaths[i].endPointTwo.second);

        shadowPathObject = (env)->NewObject(shadowPathClass,
                                            shadowClassConstructor, pointArray, startPointOne,
                                            startPointTwo, endPointOne, endPointTwo);

        (env)->SetObjectArrayElement(shadowObjectArray, i, shadowPathObject);
        (env)->DeleteLocalRef(shadowPathObject);
        (env)->DeleteLocalRef(pointArray);

    }

    return shadowObjectArray;
}

vector<pair<int, int> >
getPath(pair<int, int> src, pair<int, int> dest, set<pair<int, int> > Graph) {
    map<pair<int, int>, bool> visited;
    map<pair<int, int>, pair<int, int> > parent;
    visited.clear();
    parent.clear();

    queue<pair<int, int> > Q;
    while (!Q.empty())
        Q.pop();

    Q.push(src);

    visited[src] = true;
    parent[src] = src;

    pair<int, int> tp;
    pair<int, int> pt;
    int x, y;

    while (!Q.empty()) {
        tp = Q.front();
        Q.pop();

        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                x = tp.first + i;
                y = tp.second + j;
                pt = make_pair(x, y);
                if (Graph.find(pt) != Graph.end() && !visited[pt]) {
                    visited[pt] = true;
                    parent[pt] = tp;
                    Q.push(pt);
                }
            }
        }
    }

    vector<pair<int, int> > path;
    path.clear();

    if (!visited[dest]) {
        assert(false);
        return path;
    }

    pair<int, int> now = dest;
    while (now != parent[now]) {
        path.push_back(now);
        now = parent[now];
    }
    path.push_back(now);

    return path;
}

long double crossProduct(pair<int, int> p1, pair<int, int> p2, pair<int, int> p3) {
    long double x1, y1, x2, y2;

    x1 = p2.first - p1.first;
    y1 = p2.second - p1.second;

    x2 = p3.first - p1.first;
    y2 = p3.second - p1.second;

    long double ans = x1 * y2 - x2 * y1;

    return ans;
}

vector<pair<int, int> > getPolarOrder(vector<pair<int, int> > path, pair<int, int> ref) {
    vector<pair<pair<long double, long double>, pair<int, int> > > polarOrder;
    polarOrder.clear();

    long double angle;
    long double dist;
    long double H, B;

    int minx = 1000000009, maxx = -1000000009;

    for (auto & i : path) {
        minx = min(minx, i.first);
        maxx = max(maxx, i.first);
    }

    // LOG

    bool flag = false;

    if (ref.first >= minx && ref.first <= maxx)
        flag = true;

    for (auto & i : path) {
        H = i.second - ref.second;
        B = i.first - ref.first;

        dist = H * H + B * B;

        angle = atan(H / B);

        if (angle > 0.0 && flag)
            angle -= PI;

        polarOrder.emplace_back(make_pair(angle, dist), i);
    }

    vector<pair<pair<long double, long double>, pair<int, int> > > temp;
    temp.clear();

    for (auto & i : polarOrder) {
        if (temp.empty())
            temp.push_back(i);
        else if (temp[temp.size() - 1].first.first > i.first.first)
            temp.push_back(i);
        while (!temp.empty() && temp[temp.size() - 1].first.second > i.first.second &&
               temp[temp.size() - 1].first.first < i.first.first)
            temp.pop_back();
    }

    path.clear();
    for (auto & i : temp)
        path.push_back(i.second);

    return path;
}

long double getArea(vector<pair<int, int> > path, pair<int, int> ref) {
    path = getPolarOrder(path, ref);

    long double area = 0.0;
    for (int i = 1; i < path.size(); i++)
        area = area + crossProduct(ref, path[i - 1], path[i]);

    area = area / 2.0;
    area = abs(area);

    return area;
}

vector<pair<int, int> > getLargestComponent(set<pair<int, int> > Graph) {
    assert(!Graph.empty());

    vector<pair<int, int> > component;
    vector<pair<int, int> > now;
    component.clear();

    map<pair<int, int>, bool> visited;
    visited.clear();

    for (auto it = Graph.begin(); it != Graph.end(); it++) {
        if (visited[*it])
            continue;

        now.clear();
        now.push_back(*it);

        queue<pair<int, int> > Q;
        while (!Q.empty())
            Q.pop();

        Q.push(*it);

        visited[*it] = true;

        pair<int, int> tp;
        pair<int, int> pt;
        int x, y;

        while (!Q.empty()) {
            tp = Q.front();
            Q.pop();

            for (int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                    x = tp.first + i;
                    y = tp.second + j;
                    pt = make_pair(x, y);
                    if (Graph.find(pt) != Graph.end() && !visited[pt]) {
                        visited[pt] = true;
                        Q.push(pt);
                        now.push_back(pt);
                    }
                }
            }
        }

        if (component.size() < now.size()) {
            component.clear();
            for (const auto & i : now)
                component.push_back(i);
        }
    }

    return component;
}

vector<pair<int, int> > getOuterBoundary(pair<int, int> src, set<pair<int, int> > Graph) {
    map<pair<int, int>, bool> visited;
    visited.clear();

    vector<pair<int, int> > outerBoundary;
    outerBoundary.clear();

    queue<pair<int, int> > Q;
    while (!Q.empty())
        Q.pop();

    Q.push(src);
    visited[src] = true;

    outerBoundary.push_back(src);

    pair<int, int> tp;
    pair<int, int> pt;
    int x, y;

    while (!Q.empty()) {
        tp = Q.front();
        Q.pop();

        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                x = tp.first + i;
                y = tp.second + j;
                pt = make_pair(x, y);
                if (Graph.find(pt) != Graph.end() && !visited[pt]) {
                    visited[pt] = true;
                    Q.push(pt);
                    outerBoundary.push_back(pt);
                }
            }
        }
    }

    return outerBoundary;
}

vector<pair<int, int> > boundaryPath(const vector<pair<int, int> >& pts, pair<int, int> ref, int what) {
    set<pair<int, int> > boundary;
    boundary.clear();

    for (auto & pt : pts)
        boundary.insert(pt);

    vector<pair<long double, pair<int, int> > > polarOrder;
    polarOrder.clear();

    int minx = 1000000009, maxx = -1000000009;

    for (const auto & it : boundary) {
        minx = min(minx, it.first);
        maxx = max(maxx, it.first);
    }

    // LOG

    bool flag1 = false;

    if (ref.first >= minx && ref.first <= maxx)
        flag1 = true;

    long double angle;
    long double H, B;

    for (const auto & it : boundary) {
        H = (it.second) - ref.second;
        B = (it.first) - ref.first;
        angle = atan(H / B);

        if (angle > 0.0 && flag1)
            angle -= PI;

        polarOrder.emplace_back(angle, it);
    }

    sort(polarOrder.begin(), polarOrder.end());

    pair<int, int> tangent1, tangent2;
    tangent1 = polarOrder[0].second;
    tangent2 = polarOrder[polarOrder.size() - 1].second;

    vector<pair<int, int> > outerBoundary = getOuterBoundary(tangent1, boundary);

    boundary.clear();
    for (auto & i : outerBoundary)
        boundary.insert(i);

    vector<pair<int, int> > path1 = getPath(tangent1, tangent2, boundary);

    vector<pair<int, int> > path2;
    path2.clear();

    for (auto & i : path1)
        boundary.erase(i);

    for (const auto & it : boundary)
        path2.push_back(it);

    vector<pair<int, int> > component = getLargestComponent(boundary);

    boundary.clear();
    for (auto & i : component)
        boundary.insert(i);

    polarOrder.clear();
    for (const auto & it : boundary) {
        H = (it.second) - ref.second;
        B = (it.first) - ref.first;

        angle = atan(H / B);

        if (angle > 0.0 && flag1)
            angle -= PI;

        polarOrder.emplace_back(angle, it);
    }

    sort(polarOrder.begin(), polarOrder.end());

    tangent1 = polarOrder[0].second;
    tangent2 = polarOrder[polarOrder.size() - 1].second;

    path2 = getPath(tangent1, tangent2, boundary);

    long double area1 = getArea(path1, ref);
    long double area2 = getArea(path2, ref);

    vector<pair<int, int> > path1PolarOrder;
    vector<pair<int, int> > path2PolarOrder;
    path1PolarOrder.clear();
    path2PolarOrder.clear();

    path1PolarOrder = getPolarOrder(path1, ref);
    path2PolarOrder = getPolarOrder(path2, ref);

    vector<pair<int, int> > fullPath;
    vector<pair<int, int> > fullPathPolarOrder;
    fullPath.clear();
    fullPathPolarOrder.clear();

    for (const auto & i : path1)
        fullPath.push_back(i);

    for (int i = path2.size() - 1; i >= 0; i--)
        fullPath.push_back(path2[i]);

    for (const auto & i : path1PolarOrder)
        fullPathPolarOrder.push_back(i);

    for (int i = path2PolarOrder.size() - 1; i >= 0; i--)
        fullPathPolarOrder.push_back(path2PolarOrder[i]);

    bool flag = (area1 < area2);

    switch (what) {
    case 0: // Front Path
        return flag ? path1 : path2;
    case 1: // Front Polar Path
        return flag ? path1PolarOrder : path2PolarOrder;
    case 2: // Back Path
        return flag ? path2 : path1;
    case 3: // Back Polar Path
        return flag ? path2PolarOrder : path1PolarOrder;
    case 4: // Full Path
        return fullPath;
    case 5: // Full Polar Path
        return fullPathPolarOrder;
    }
}

double getDistance(pair<int, int> pointOne, pair<int, int> pointTwo) {
    return (pow((pointOne.first - pointTwo.first), 2) +
            pow((pointOne.second - pointTwo.second), 2));
}

pair<int, int> getClosestPointToLight(const vector<pair<int, int> >& points, pair<int, int> ref) {
    pair<int, int> closestPoint;

    double minDistance = INT_MAX;

    for (auto & point : points) {
        double distance = getDistance(ref, point);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = point;
        }
    }

    return closestPoint;
}

ShadowPath
getShadowPathsFromContour(vector<pair<int, int> > points, int width, int height, float angle,
                          int shadowLength, pair<int, int> ref) {
    vector<pair<int, int> > boundary_front_polar = boundaryPath(points, ref, 1);

    int boundary_front_polar_size = boundary_front_polar.size();

    // Translate points away from light to avoid shadow out of the contour towards light source
    for (int i = 0; i < boundary_front_polar_size; i++) {
        boundary_front_polar[i].first += CORRECTIVE_OFFSET * cos(angle * PI / 180);
        boundary_front_polar[i].second += CORRECTIVE_OFFSET * sin(angle * PI / 180);
    }

    vector<pair<int, int> > pathPoints;

    for (const auto & i : boundary_front_polar) {
        pathPoints.push_back(i);
    }

    pair<int, int> translatedPointOne = boundary_front_polar[0];
    pair<int, int> translatedPointTwo = boundary_front_polar[boundary_front_polar_size - 1];

    translatedPointOne.first += shadowLength * cos(angle * PI / 180);
    translatedPointOne.second += shadowLength * sin(angle * PI / 180);

    translatedPointTwo.first += shadowLength * cos(angle * PI / 180);
    translatedPointTwo.second += shadowLength * sin(angle * PI / 180);

    pathPoints.push_back(translatedPointTwo);
    pathPoints.push_back(translatedPointOne);

    ShadowPath shadowPath;
    shadowPath.points = pathPoints;

    pair<int, int> closestPointToLight = getClosestPointToLight(boundary_front_polar, ref);

    if ((closestPointToLight.first == boundary_front_polar[0].first &&
         closestPointToLight.second == boundary_front_polar[0].second) ||
        (closestPointToLight.first == boundary_front_polar[boundary_front_polar_size - 1].first &&
         closestPointToLight.second ==
         boundary_front_polar[boundary_front_polar_size - 1].second)) {

        shadowPath.startPointOne = boundary_front_polar[0];
        shadowPath.startPointTwo = boundary_front_polar[boundary_front_polar_size - 1];
    } else {
        shadowPath.startPointOne = closestPointToLight;
        shadowPath.startPointTwo = closestPointToLight;
    }

    shadowPath.endPointOne = translatedPointOne;
    shadowPath.endPointTwo = translatedPointTwo;

    return shadowPath;
}

pair<int, int> getReferencePointFromContour(const vector<pair<int, int> >& contour, float angle) {
    int x = 0, y = 0;

    for (auto & i : contour) {
        x += i.first;
        y += i.second;
    }

    x = (int) (x / contour.size());
    y = (int) (y / contour.size());

    return make_pair(x - (REFERENCE_POINT_OFFSET * cos(angle * PI / 180)),
                     y - (REFERENCE_POINT_OFFSET * sin(angle * PI / 180)));
}

extern "C"
JNIEXPORT jobjectArray JNICALL
Java_com_sdsmdg_harjot_longshadows_shadowutils_LongShadowsGenerator_getShadowPaths(
        JNIEnv *env,
        jobject,
        jintArray arr,
        jint width,
        jint height,
        jfloatArray angles,
        jint numAngles,
        jintArray shadowLengths,
        jint backgroundColor) {

    vector<vector<pair<int, int> > > ans;
    ans.clear();

    jint *c_array = (env)->GetIntArrayElements(arr, nullptr);
    jfloat *angles_array = (env)->GetFloatArrayElements(angles, nullptr);
    jint *shadow_lengths = (env)->GetIntArrayElements(shadowLengths, nullptr);
    ans = contours(env, c_array, width, height, backgroundColor);

    vector<ShadowPath> shadowPaths;
    for (auto & an : ans) {
        for (int j = 0; j < numAngles; j++) {
            shadowPaths.push_back(
                    getShadowPathsFromContour(an, width, height, angles_array[j],
                                              shadow_lengths[j],
                                              getReferencePointFromContour(an,
                                                                           angles_array[j])));
        }
    }

    return convertToObjectArray(env, shadowPaths);
}
