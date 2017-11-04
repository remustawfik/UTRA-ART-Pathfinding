// TODO:
// Dynamically resize the path planning grid to fit the goal location?

#include <vector>
#include <limits>
#include <queue>
#include <algorithm>
#include <math.h>
#include "path_planner.h"

using namespace std;

// Convert degrees (in the format that we get raw from the GPS) into radians
double degreesToRadians(int degrees, double arcMinutes, bool positive)
{
    double totalDegrees = degrees + (arcMinutes/60.0);
    if (!positive)
    {
        totalDegrees *= -1;
    }

    return totalDegrees / 180.0 * PI;
}

// Convert from raw gps structure to the useful internal representation (radians)
GpsInfo convertGpsInfoToRadians(const RawGpsInfo& info)
{
    double latitude = degreesToRadians(info.latDegrees, info.latArcMinutes, info.north);
    double longitude = degreesToRadians(info.lonDegrees, info.lonArcMinutes, info.east);
    return GpsInfo(latitude, longitude);
}

// Returns the distance (metres) and bearing (radians) to get *from* a *to* b
// Bearing of 0 is north, positive means turn due east, negative means turn due west
// Uses: http://www.movable-type.co.uk/scripts/latlong.html
// Uses the equirectangular approximation, since we're talking about extremely small distances
void gpsDelta(const GpsInfo& a, const GpsInfo& b, double& metres, double& bearing)
{
    // There might be some wrap-around issues here, but, like, we're not going to be anywhere
    // near Greenwich, so we should be fine :)
    // PS these values are in radians
    double latDelta = b.latitude - a.latitude;
    double lonDelta = b.longitude - a.longitude;
    double averageLat = (a.latitude + b.latitude) / 2;

    const double radiusOfEarth = 6371000; // metres

    double x = latDelta; // This is actually up-down, but in the context of our north-is-zero coordinate system, it makes most sense to call this the x axis
    double y = lonDelta * cos(averageLat);

    metres = sqrt(x*x + y*y) * radiusOfEarth;

    bearing = atan2(y, x);
}

// Find the distance between two points
double getDistance(Point& a, Point& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Expand the obstalces in the given grid by the given amount
void expandObstacles(nav_msgs::OccupancyGrid& obstacles, int amount)
{
    cout << "expanding amount is: " << amount << endl;
    int xSize = obstacles.info.width;
    int ySize = obstacles.info.height;

    // Do a breadth first search (BFS)
    vector<Point> thisLevel;
    vector<Point> nextLevel;

    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            // Initialize the BFS with all of the obstacles
            if (at(obstacles, x, y) != 0)
            {
                //cout << "FOUND A THING" << endl;
                thisLevel.push_back(Point(x, y));
            }
        }
    }

    for (int dist = 0; dist < amount; dist++)
    {

        // Iterate over everything in this level
        vector<Point>::iterator it = thisLevel.begin();
        vector<Point>::iterator end = thisLevel.end();
        for (; it != end; ++it)
        {
            Point p = *it;

            // Now iterate over this point's neighbours
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    // Don't consider this point a neighbour of itself :)
                    if (i == 0 && j == 0) continue;

                    Point n(p.x + i, p.y + j);

                    // Ignore points that are outside the map
                    if (n.x < 0 || n.y < 0 || n.x >= xSize || n.y >= ySize) continue;

                    // Fill in the neighbour if not already filled
                    if (at(obstacles, n.x, n.y) == 0)
                    {
                        //cout << "expanded a cell!" << endl;
                        
                        at(obstacles, n.x, n.y) = occupancyGridOccupied;

                        // Add neighbour to the set of points to explore next iteration of BFS
                        nextLevel.push_back(n);
                    }
                }
            }
        }

        thisLevel = nextLevel;
        nextLevel.clear();
    }
}


// Calculate the distance between each cell in the grid and the nearest obstacle
// Note that this function considers "one cell" in any direction (including diagonals)
// the same distance. Which isn't exactly true, but it's Good Enough.
Map2D<int> getDistanceMap(const nav_msgs::OccupancyGrid& obstacles, int limit)
{
    int xSize = obstacles.info.width;
    int ySize = obstacles.info.height;
    cout << "before" << endl;
    // This is the map which will contain the distances.
    Map2D<int> distMap(xSize, ySize, maxInt);
    cout << "after" << endl;
    // Do a breadth first search (BFS)
    vector<Point> thisLevel;
    vector<Point> nextLevel;

    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            // Initialize the BFS with all of the obstacles
            if (at(obstacles, x, y) != 0)
            {
                thisLevel.push_back(Point(x, y));
            }
        }
    }

    // Only calculate distances up to the limit distance
    for (int dist = 0; dist <= limit; dist++)
    {

        // Iterate over everything in this level
        vector<Point>::iterator it = thisLevel.begin();
        vector<Point>::iterator end = thisLevel.end();
        for (; it != end; ++it)
        {
            Point p = *it;

            // Ignore points that are outside the map
            if (p.x < 0 || p.y < 0 || p.x >= xSize || p.y >= ySize)
            {
                continue;
            }

            // Update the current point's distance if applicable
            if (dist < distMap.get(p.x, p.y))
            {
                distMap.set(p.x, p.y, dist);
                
                // Now iterate over this point's neighbours
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        // Don't consider this point a neighbour of itself :)
                        if (i == 0 && j == 0) continue;

                        // Add neighbour to the set of points to explore next iteration of BFS
                        nextLevel.push_back(Point(p.x + i, p.y + j));
                    }
                }
            }
        }

        thisLevel = nextLevel;
        nextLevel.clear();
    }

    return distMap;
}

// Generate a map containing the cost of entering each cell in the map
// Based on distance to nearest obstacle
// Inverse-square cost
Map2D<double> getCostMap(Map2D<int> distMap)
{
    int xSize = distMap.getXSize();
    int ySize = distMap.getYSize();

    Map2D<double> costMap(xSize, ySize, 0);

    // Just iterate over the distance map and calculate the corresponding cost for each cell
    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            int dist = distMap.get(x, y);
            double cost = 0;

            if (useCostFunction)
            {
                // if (dist != 0 && dist != maxInt)
                // {
                //     // Inverse square cost
                //     cost = 300.0 / (dist*dist);
                // }

                if (dist <= 10)
                {
                    cost = 50.0;
                }
            }
            
            costMap.set(x, y, cost);
        }
    }

    return costMap;
}

geometry_msgs::Point32 convertPoint(const Point& p)
{
    geometry_msgs::Point32 pFloat;
    pFloat.x = p.x;
    pFloat.y = p.y;

    return pFloat;
}

geometry_msgs::PointStamped createPointMessage(const Point& p)
{
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = "/map";
    msg.point.x = p.x;
    msg.point.y = p.y;

    return msg;
}

// Convert a Path (which is just a simple vector of points) into a ROS
// nav_msgs::Path message which can then be published
nav_msgs::Path createPathMessage(const Path& path)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "/map";
    
    for (Path::const_iterator it = path.begin(); it != path.end(); it++)
    {
        const Point& p = *it;
        
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "/map";
        poseStamped.pose.position.x = p.x;
        poseStamped.pose.position.y = p.y;

        msg.poses.push_back(poseStamped);
    }

    return msg;
}

visualization_msgs::Marker createLineVisualizationMessage(const vector<geometry_msgs::Point32>& points, const conversionParams& params)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/map";
    msg.ns = "vision_lines";
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.id = 0;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = 1; // line thickness
    msg.color.b = 1; // color is blue
    msg.color.a = 1; // alpha is 100% (no transparency)

    for (vector<geometry_msgs::Point32>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::Point p;

        // Convert points to cell locations
        p.x = (it->x / params.cellSize) + params.xOffset;
        p.y = (it->y / params.cellSize) + params.yOffset;

        msg.points.push_back(p);
    }

    return msg;
}

        

// At the end of A*, we need to reconstruct the path by working backwards from the end point
Path reconstructPath(Map2D<Point>& cameFrom, Point start, Point end)
{
    Path path;
    
    // Start at the end point, work backwards
    Point current = end;
    path.push_back(current);

    // Iterate until we get to the start point
    while (current != start)
    {
        // We are currently in the 'current' grid cell. How did the robot get here?
        // Let's check the cameFrom map to see!
        current = cameFrom.get(current.x, current.y);
        path.push_back(current);
    }

    // Flip the path since we recosntructed it in reverse
    reverse(path.begin(), path.end());

    return path;
}

void initializeOccupancyGrid(nav_msgs::OccupancyGrid& grid, int width, int height, int value)
{
    grid.header.frame_id = "/map";
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1;
    grid.data.resize(grid.info.width * grid.info.height);

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            at(grid, i, j) = value;
        }
    }
}


// The A* implementation
Path doAStar(Map2D<double>& costMap, const nav_msgs::OccupancyGrid& obstacleMap, Point start, const Goal& goal)
{
    int xSize = costMap.getXSize();
    int ySize = costMap.getYSize();

    // Initialize some data structures we'll use to keep track of stuff
    Map2D<bool> visited(xSize, ySize, false);
    Map2D<double> cumulativeScore(xSize, ySize, infinity);
    Map2D<double> estimatedTotalScore(xSize, ySize, infinity);
    Map2D<Point> cameFrom(xSize, ySize);
    scoredQueue pq;

    // Add the first point (the starting point)
    cumulativeScore.set(start.x, start.y, 0);
    double estimatedScore = goal.getHeuristicDistanceFrom(start);
    estimatedTotalScore.set(start.x, start.y, estimatedScore);
    pq.push(scoredPoint(estimatedScore, start));

    // Iterate until the priority queue is empty (or until we find a path and break out early)
    while (!pq.empty())
    {
        // Pop the point with the lowest score off the queue
        scoredPoint sp = pq.top();
        pq.pop();
        Point p = sp.second;

        // Check if we've visited this node before, and if so skip it
        if (visited.get(p.x, p.y))
        {
            continue;
        }

        // Okay, we haven't seen this point before. Let's visit it now.
        visited.set(p.x, p.y, true);

        // Check if we're at the goal -- if so, reconstruct the path and return it
        if (goal.contains(p))
        {
            return reconstructPath(cameFrom, start, p);
        }

        // Look up the cost to get to this point so far
        double cScore = cumulativeScore.get(p.x, p.y);

        // Alright, find this point's neighbours
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                Point neighbour(p.x + i, p.y + j);

                // Don't consider the current point a neighbour, obviously
                if (neighbour == p)
                {
                    continue;
                }

                // Don't consider anything that's off the map
                if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= xSize || neighbour.y >= ySize)
                {
                    continue;
                }

                // Don't consider anything that's an obstacle
                if (at(obstacleMap, neighbour.x, neighbour.y) != 0)
                {
                    continue;
                }

                // Don't consider anything that's already been visited
                if (visited.get(neighbour.x, neighbour.y))
                {
                    continue;
                }

                // Calculate this neighbour's tentative new score if we were to travel to it from the current point
                bool isOnDiagonal = (abs(i) == 1 && abs(j) == 1);
                double distanceCost = (isOnDiagonal ? sqrt(2) : 1.0);
                double obstacleCost = costMap.get(neighbour.x, neighbour.y);
                double tentativeCScore = cScore + distanceCost + obstacleCost;

                // If this new score is better than anything we've seen for this neighbour,
                // then update the neighbour
                if (tentativeCScore < cumulativeScore.get(neighbour.x, neighbour.y))
                {
                    cumulativeScore.set(neighbour.x, neighbour.y, tentativeCScore);
                    double estimatedScore = tentativeCScore + goal.getHeuristicDistanceFrom(neighbour);
                    estimatedTotalScore.set(neighbour.x, neighbour.y, estimatedScore);
                    cameFrom.set(neighbour.x, neighbour.y, p);
                    pq.push(scoredPoint(estimatedScore, neighbour));
                }
            }
        }
    }
         
    // Huh? We couldn't find a way to get to the goal! Just return an empty path.
    return Path();
}

void addBlinders(nav_msgs::OccupancyGrid& g, Point currentPosition, double angleD, int displacement)
{
    int startX = currentPosition.x - displacement;
    int startY = currentPosition.y;
    
    double angleR = angleD * PI / 180;

    double slope = tan(angleR);

    for (int y = 0; y < g.info.height; y++)
    {
        int deltaY = abs(y - startY);
        double deltaX = deltaY * slope;

        int endX = startX + deltaX;

        // Make sure endX is in the range [0, g.info.width]
        if (endX < 0) endX = 0;
        if (endX > g.info.width) endX = g.info.width;
        
        for (int x = 0; x < endX; x++)
        {
            at(g, x, y) = occupancyGridOccupied;
        }
    }
}

// Calculate the distance map and then the cost map
// Then do A*
Path findPath(nav_msgs::OccupancyGrid& g, Point currentPosition, const Goal& goal)
{
    if (goal.isOutOfBounds(g))
    {
        cout << "Oops, goal (type " << goal.getGoalType() << ") was outside the occupancy grid! Forget it ..." << endl;
        Path emptyPath;
        return emptyPath;
    }
    cout << "Expanding obstacles ..." << endl;
    expandObstacles(g, obstacleExpansionDistance);
    cout << "Computing distance map .." << endl;
    Map2D<int> distanceMap = getDistanceMap(g);
    cout << "Computing cost map ..." << endl;
    Map2D<double> costMap = getCostMap(distanceMap);

    if (useBlinders)
    {
        // NOTE: blinders must be added *after* doing obstacle expansion and cost map,
        // because we don't want to expand the blinders or associate a cost with being
        // near them. We just don't want the rover to think it can get around any obstalce
        // by driving through its blind spots :)
        cout << "Adding blinders ..." << endl;
        addBlinders(g, currentPosition, blindersAngleD, blindersDisplacement);
    }

    Path path;
    
    // If the current goal point is inside an obstacle, we'll never be able to get to it!
    // Jiggle it around until it's outside
    Goal* adjustedGoal = goal.clone();
    if (adjustedGoal->isInsideObstacles(g))
    {
        cout << "Trying to remove goal from obstacles ..." << endl;
        adjustedGoal->moveOutsideOfObstacles(currentPosition, g);
    }

    // Check if we goal is actually outside obstacle now
    if (adjustedGoal->isInsideObstacles(g))
    {
        // Oh noes! Still inside an obstacle!
        cout << "Couldn't remove goal from obstacles! Bailing!" << endl;
    }
    else
    {
        cout << "Running A* ..." << endl;
        path = doAStar(costMap, g, currentPosition, *adjustedGoal);
    }
    
    delete adjustedGoal;
    return path;
}

// Quick function to check if both val1 and val2 are in the range [0, size)
void boundsCheck(int val1, int val2, int size)
{
    if (!(val1>=0 && val1<size && val2>=0 && val2<size))
    {
        throw outOfBounds;
    }
}

Point getGoalPointFromSlam(Point& origin, conversionParams& params)
{
    double xDelta = globalGoalPositionSLAM.x - globalPositionSLAM.x;
    double yDelta = globalGoalPositionSLAM.y - globalPositionSLAM.y;
    double deltaDistance = sqrt(xDelta*xDelta + yDelta*yDelta);

    double bearing = atan2(yDelta, xDelta) + globalBearingSLAM;

    // Calculate how far up and to the right of the robot's current location the goal lies
    double metresUp = cos(bearing) * deltaDistance;
    double metresRight = sin(bearing) * deltaDistance;

    int cellsUp = (int)(round(metresUp / params.cellSize));
    int cellsRight = (int)(round(metresRight / params.cellSize));

    return Point(origin.x + cellsRight, origin.y + cellsUp);
}
    
// For a robot located in cell 'origin' of a grid, with grid parameters 'params',
// where robot is located at gps location robotLocation, and goal is goalLocation, and
// positive y direction (up) on the grid is assumed to be in the direction the robot is
// facing, and assuming that this direction is given by 'bearing' in radians east of north,
// then this function returns the grid coordinates of the goal point.
// Assume that when we say the robot is at the point origin, we mean that it is centred in
// this cell of the grid. And we want to find the cell in which the goal point lies.
Point getGoalPoint(Point& origin, conversionParams& params, const GpsInfo& robotLocation, const GpsInfo& goalLocation, double bearing)
{
    // Find the distance and bearing from the current gps location to the goal point
    double gpsDeltaDistance, gpsDeltaBearing;
    gpsDelta(robotLocation, goalLocation, gpsDeltaDistance, gpsDeltaBearing);

    // Take into account the fact that the robot is not necessarily facing north
    gpsDeltaBearing -= bearing;

    // Calculate how far up and to the right of the robot's current location the goal lies
    double metresUp = cos(gpsDeltaBearing) * gpsDeltaDistance;
    double metresRight = sin(gpsDeltaBearing) * gpsDeltaDistance;

    int cellsUp = (int)(round(metresUp / params.cellSize));
    int cellsRight = (int)(round(metresRight / params.cellSize));

    return Point(origin.x + cellsRight, origin.y + cellsUp);
}

// Use the given conversion parameters to go from distances (metres) stored in p
// to an actual cell index in the grid
Point toMap(const geometry_msgs::Point32 p, const conversionParams& params)
{
    int x = (int)((p.x / params.cellSize) + params.xOffset);
    int y = (int)((p.y / params.cellSize) + params.yOffset);

    return Point(x, y);
}    

// Check if the line defined by start -> end fits inside the width described by the given
// width and height. If not, then attempt to clip the line segment to show only the portion
// that fits inside the window.
// Returns true if the line fits in the window, or could be clipped to fit in the window.
// Returns false if the entire line lies outside the window.
// Assumes that the points start and end have already been converted from distances (metres)
// to cell numbers. So if start = (5, 6) that means the cell map[5][6], and *not* the points
// 5m right and 6m up of the origin.
bool clipToWindow(Pfloat& start, Pfloat& end, int width, int height)
{
    const double epsilon = 0.0001;

    // Clip x < 0
    if (start.x < 0 && end.x < 0)
    {
        return false;
    }
    else if (start.x < 0)
    {
        double slope = (end.y - start.y) / (end.x - start.x);
        double clipAmount = 0 - start.x;
        start.x = 0;
        start.y += clipAmount * slope;
    }
    else if (end.x < 0)
    {
        double slope = (start.y - end.y) / (start.x - end.x);
        double clipAmount = 0 - end.x;
        end.x = 0;
        end.y += clipAmount * slope;
    }

    // Clip y < 0
    if (start.y < 0 && end.y < 0)
    {
        return false;
    }
    else if (start.y < 0)
    {
        double slope = (end.x - start.x) / (end.y - start.y);
        double clipAmount = 0 - start.y;
        start.y = 0;
        start.x += clipAmount * slope;
    }
    else if (end.y < 0)
    {
        double slope = (start.x - end.x) / (start.y - end.y);
        double clipAmount = 0 - end.y;
        end.y = 0;
        end.x += clipAmount * slope;
    }

    // Clip x >= width
    // Note: since we're trying to clip so that x < width, we use a small value
    // epsilon, and clip so that x = width - epsilon
    if (start.x > (width - epsilon) && end.x > (width - epsilon))
    {
        return false;
    }
    else if (start.x > (width - epsilon))
    {
        double slope = (end.y - start.y) / (start.x - end.x);
        double clipAmount = start.x - (width - epsilon);
        start.x = width - epsilon;
        start.y += clipAmount * slope;
    }
    else if (end.x > (width - epsilon))
    {
        double slope = (start.y - end.y) / (end.x - start.x);
        double clipAmount = end.x - (width - epsilon);
        end.x = width - epsilon;
        end.y += clipAmount * slope;
    }

    // Clip y >= height
    // Note: since we're trying to clip so that y < width, we use a small value
    // epsilon, and clip so that y = height - epsilon
    if (start.y > (height - epsilon) && end.y > (height - epsilon))
    {
        return false;
    }
    else if (start.y > (height - epsilon))
    {
        double slope = (end.x - start.x) / (start.y - end.y);
        double clipAmount = start.y - (height - epsilon);
        start.y = height - epsilon;
        start.x += clipAmount * slope;
    }
    else if (end.y > (height - epsilon))
    {
        double slope = (start.x - end.x) / (end.y - start.y);
        double clipAmount = end.y - (height - epsilon);
        end.y = height - epsilon;
        end.x += clipAmount * slope;
    }

    return true;
}

// Consider the line segment defined by start -> end
// Find all cells in the map which this segment touches, and set those cells to the value fillVal
// Note that this function expects that the given start and end points are expressed as distances,
// (i.e. if start = (5, 6) that means that the point is 5m right and 6m up of the origin)
// You need to pass in some conversion params to tell this function how to convert these distances
// to actual cell locations in the map
void fillOccupancyGrid(nav_msgs::OccupancyGrid& map, Pfloat start, Pfloat end, int fillVal, const conversionParams& params)
{
    cout << "Line before: (" << start.x << ", " << start.y << ") -> (" << end.x << ", " << end.y << ")" << endl;

    // Convert points to cell locations
    start.x = (start.x / params.cellSize) + params.xOffset;
    end.x = (end.x / params.cellSize) + params.xOffset;
    start.y = (start.y / params.cellSize) + params.yOffset;
    end.y = (end.y / params.cellSize) + params.yOffset;

    cout << "Line after: (" << start.x << ", " << start.y << ") -> (" << end.x << ", " << end.y << ")" << endl;

    // If the line segment lies completely or partially outside of the map, attempt to clip
    // it so that we only get the section inside the map
    bool success = clipToWindow(start, end, map.info.width, map.info.height);

    // Oops, the line was completely outside the map! There's nothing we can do, so just return
    if (!success)
    {
        cout << "Warning: you specified a line which was competely outside of the window." << endl;
        cout << "Skipping that line..." << endl;
        return;
    }

    int xStart = (int)start.x;
    int xEnd = (int)end.x;
    int yStart = (int)start.y;
    int yEnd = (int)end.y;

    // Double check to make sure that both start and end lie inside the map
    // Should be because that's what clipToWindow is supposed to do :P
    boundsCheck(xStart, xEnd, map.info.width);
    boundsCheck(yStart, yEnd, map.info.height);

    // Special case when xStart == xEnd
    if (xStart == xEnd)
    {
        // Swap start and end so that yStart < yEnd
        if (yEnd < yStart)
        {
            int temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        }

        // Just iterate over all y values touched by the line
        for (int y = yStart; y <= yEnd; y++)
        {
            at(map, xStart, y) = fillVal;
        }

        return;
    }

    // Okay, general case here

    // Swap start and end so that xStart < xEnd
    if (xEnd < xStart)
    {
        int temp = xStart;
        xStart = xEnd;
        xEnd = temp;

        temp = yStart;
        yStart = yEnd;
        yEnd = temp;

        Pfloat tempP = start;
        start = end;
        end = tempP;
    }

    double slope = (end.y - start.y) / (end.x - start.x);

    // For each x column that this line touches, we want to figure out which y cells in that column
    // are hit by the line
    for (int x = xStart; x <= xEnd; x++)
    {
        int xNext = x + 1;
        double xDelta = x - start.x;
        double xNextDelta = xNext - start.x;

        // Calculate the range of affected y cells for this x column
        int yStart = (int)((x == xStart) ? start.y : start.y + (xDelta*slope));
        int yEnd = (int)((x == xEnd) ? end.y : start.y + (xNextDelta*slope));

        // Flip start and end so that yStart < yEnd
        if (yEnd < yStart)
        {
            int temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        }
        
        // Fill all of the affect y cells
        for (int y = yStart; y <= yEnd; y++)
        {
            at(map, x, y) =  fillVal;
        }
    }
}

// http://stackoverflow.com/questions/9296059/read-pixel-value-in-bmp-file
// Draws the path that your function calculated onto the input image. Essentially a way
// to visualize the path. You must specify a blank input image to read, the output image,
// and the path. This function will essentially copy the input image onto the output
// image, but will superimpose the path on top in black.
// Why do you need to specify a blank input image??
// Because I couldn't be bothered to figure out the file format of bitmaps beyond what I needed
// to do this, so I don't actually know how to write a complete proper bitmap file header :)
// So I need a template file to copy it from
void addObstaclesAndPathToMap(const char* inputFile, const char* outputFile, const nav_msgs::OccupancyGrid& m, const vector<geometry_msgs::Point32>& psFloat, Path& path, const conversionParams& params)
{
    vector<Point> ps;

    // First, just iterate over all of the input points specified  (which should be in distance format) and
    // convert them into actual grid cell format
    for (int i = 0; i < psFloat.size(); i++)
    {
        ps.push_back(toMap(psFloat[i], params));
    }

    // Open files
    FILE* in = fopen(inputFile, "rb");
    if (!in)
    {
        throw cantReadFile;
    }
    FILE* out = fopen(outputFile, "wb");
    if (!out)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, in);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));

    // Width and height of template file should match width and height of the map we're given
    if (width != m.info.width || height != m.info.height)
    {
        return;
    }

    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    // Now let's create a temporary map, and use it to store which cells are in the path
    Map pathCells(width, height);
    for (int i = 0; i < path.size(); i++)
    {
        Point p = path[i];        
        if (p.x >= pathCells.getXSize() || p.y >= pathCells.getYSize())
        {
            cout << "Hmmm ... your path seems to be too big to fit onto this bitmap!" << endl;
        }
        //cout << "Path point: (" << (int)p.x << ", " << (int)p.y << ")" << endl;
        pathCells.set(p.x, p.y, true);
    }

    // Copy the header verbatim into the output file
    fwrite(header, sizeof(unsigned char), 54, out);

    // Now iterate over the input image and copy it into the output image along with the path
    for (int y = 0; y < height; y++)
    {
        // Read a row from the input image
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, in);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            if (std::find(ps.begin(), ps.end(), Point(x, y)) != ps.end())
            {
                // If this pixel is a one of the points, set it to red
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 255; // red
            }
            else if (pathCells.get(x, y))
            {
                // If this pixel is in the path, set it to blue
                row[x*3] = 255; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
            else if (at(m, x, y) != 0)
            {
                // If this pixel is an obstacle, set it black
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
        }

        // Spit the row back out to the output image
        fwrite(row, sizeof(unsigned char), bytesPerRow, out);
    }

    fclose(in);
    fclose(out);
}

// Return by reference the width and height of the bitmap specified by filename
// Must be a 24-bit bitmap file format
void getSizeOfBitmap(const char* filename, int& width, int& height)
{
    FILE* f = fopen(filename, "rb");
    if (!f)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, f);
    
    // Extract width & height
    width = *((int*)(header + 18));
    height = *((int*)(header + 22));
}

// Callback for when we receive data from the GPS
// Assumes that the message is sent as a point, and the latitude
// is stored in radians in the x coordinate of the point, and the
// longitude is stored in radians in the y coordinate of the point.
// Yes, major hack, I know.
void gpsCallback(const geometry_msgs::Point32::ConstPtr& gpsLatLon)
{
    // Just set the global "current gps location"
    globalGpsLocation.latitude = gpsLatLon->x;
    globalGpsLocation.longitude = gpsLatLon->y;
}

// Callback for when we receive data from the IMU
void imuCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // Just set the global "current imu bearing"
    globalBearing = msg->data;
}

void resetSlam()
{
    std_msgs::String msg;
    msg.data = "reset";
    slamCommandsPublisher.publish(msg);
}

// Right now this is really dumb.
// It could be smarter in the future.
void updateSlamGoal()
{
    // These are all measurements in metres
    double deltaX = globalGoalPositionSLAM.x - globalPositionSLAM.x;
    double deltaY = globalGoalPositionSLAM.y - globalPositionSLAM.y;
    double distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    cout << "Distance from us ("
         << globalPositionSLAM.x << ", "
         << globalPositionSLAM.y << ") to goal ("
         << globalGoalPositionSLAM.x << ", "
         << globalGoalPositionSLAM.y << ") be like: "
         << distance << endl;

    // Check if we've reached our destination yet
    if (distance < reachedGoalDistance)
    {
        cout << "We got to the goal!" << endl;
        // Set the goal to be 1 metres in front of us
        globalGoalPositionSLAM.x = 0;
        globalGoalPositionSLAM.y = 1;
        globalGoalPositionSLAM.z = 0;

        // Reset our current position
        globalPositionSLAM.x = 0;
        globalPositionSLAM.y = 0;
        globalPositionSLAM.z = 0;
        globalBearingSLAM = 0;

        // Reset SLAM
        resetSlam();

        // We may have to ignore the next few messages coming in from SLAM at this point
        // Don't really know yet; maybe it won't be an issue?
    }
}


// Callback for when we receive processed data about obstacles from the 
// camera. This callback not currently used, currently expecting to
// pass a Polygon of points instead.
void camCallbackOccupancyGrid(const nav_msgs::OccupancyGrid::Ptr& m)
{
    // NOTE the input file must actually exist! It's supposed to be just a blank
    // bitmap (24-bit colour). Make sure it's big enough. Mine is size approx 1000x600 or something
    const char* input = "/home/art/Desktop/blank.bmp";
    const char* output = "/home/art/Desktop/lines.bmp";

    // For now, we are just outputting the path we find to bitmap,
    // so read the target bitmap's width and height
    // int width, height;
    // getSizeOfBitmap(input, width, height);

    int width = m->info.width;
    int height = m->info.height;

    // // Those better match the grid's measurements
    // if (m->info.width != width || m->info.height != height)
    // {
    //     cout << "Oops, grid was the wrong size. Should be:" << endl;
    //     cout << width << " x " << height << endl;
    //     cout << "Ignoring ... " << endl;
    // }

    // Find where the occupancy grid is saying the origin is, and use that info
    // to construct conversion params from metres -> cells
    double x = m->info.width / 2 + 0.5;
    double y = m->info.height / 2 + 0.5;
    conversionParams mapServerParams(x, y, 1.0, height, width);
    conversionParams slamParams(x, y, 0.05, height, width);
    
    // Calculate where the goal point is on the grid
    Point currentCell((int)x, (int)y);

    // This is the GPS way of getting the goal point. Only basic unit testing, no integration tests.
    //Point goalCell = getGoalPoint(currentCell, mapServerParams, globalGpsLocation, globalGpsGoalLocation, globalBearing);
    // This is the SLAM way of getting the goal point. Yaw tested, works. Position not tested.
    updateSlamGoal();
    Point goalCell = getGoalPointFromSlam(currentCell, slamParams);

    // Do A* n' stuff
    Path path = findPath(*m, currentCell, PointGoal(goalCell.x, goalCell.y));

    // Publish start & end points
    currentPointPublisher.publish(createPointMessage(currentCell));
    goalPointPublisher.publish(createPointMessage(goalCell));
    
    // At this point, the map is expanded. Let's republish!
    cout << "Publishing expanded obstacles" << endl;
    mapPublisher.publish(*m);

    // Draw stuff onto the bitmap
    //const vector<geometry_msgs::Point32> points; // empty; does not apply in this case
    //addObstaclesAndPathToMap(input, output, *m, points, path, p);
    
    // Publish the resulting path we found
    pathPublisher.publish(createPathMessage(path));
}


// Callback for when we receive processed data about obstacles from the 
// camera. 
void camCallbackLines(const geometry_msgs::Polygon::ConstPtr& msg)
{
    cout << "Start!" << endl;

    // NOTE the input file must actually exist! It's supposed to be just a blank
    // bitmap (24-bit colour). Make sure it's big enough. Mine is size approx 1000x600 or something
    const char* input = "/home/art/Desktop/blank.bmp";
    const char* output = "/home/art/Desktop/lines.bmp";

    // extract the list of points in the message
    const vector<geometry_msgs::Point32>& points = msg->points;

    // we expect each pair of points to represent a line segment.
    // So if there are an odd number of points, then there's something wrong
    // So just ignore the message and print error message
    if (points.size() % 2 != 0)
    {
        cout << "Oops, ignoring incoming message because the number of points isn't even!" << endl;
        return;
    }

    // For now, we are just outputting the path we find to bitmap,
    // so read the target bitmap's width and height
    int width, height;
    getSizeOfBitmap(input, width, height);

    // Optimization: can we just have a single static map,
    // and then before we fill the new lines, we fill the old ones with 0?
    // My guess is it would be faster, but maybe it depends how efficient
    // the memory allocation stuff is. Iunno.

    cout << "Building occupancy grid ..." << endl;

    nav_msgs::OccupancyGrid m;
    initializeOccupancyGrid(m, width, height, 0);

    double x = width/2;// + 0.5;
    double y = height/2;// + 0.5;
    //conversionParams p(x, y, 0.1, height, width);
    //conversionParams p(x - 160, y - 120, 1.0, height, width);
    conversionParams p(x, y, 1.0, height, width);

    // Based on the line segments we are given, actually fill the cells in the occupancy grid which correspond to obstacles
    for (int i = 0; i < points.size() / 2; i++)
    {
        fillOccupancyGrid(m, points[2*i], points[2*i+1], occupancyGridOccupied, p);
    }

    cout << "Calculating goal point ..." << endl;
    Point currentCell((int)x, (int)y);
    Point goalCell = getGoalPoint(currentCell, p, globalGpsLocation, globalGpsGoalLocation, globalBearing);

    // Do A* n' stuff
    Path path = findPath(m, currentCell, PointGoal(goalCell.x, goalCell.y));

    cout << "LOL, not creating bitmap output ..." << endl;
    //addObstaclesAndPathToMap(input, output, m, points, path, p);

    cout << "Publishing current and goal points ..." << endl;
    currentPointPublisher.publish(createPointMessage(currentCell));
    goalPointPublisher.publish(createPointMessage(goalCell));

    cout << "Publishing lines from vision ..." << endl;
    linesPublisher.publish(createLineVisualizationMessage(points, p));

    cout << "Enhancing & publishing occupancy grid ..." << endl;
    //expandObstacles(m, 2); // Flesh out the obstacles a bit for easier viewing
    mapPublisher.publish(m);

    cout << "Publishing path ..." << endl;
    pathPublisher.publish(createPathMessage(path));

    cout << "Done!" << endl;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    const geometry_msgs::Quaternion orientation = msg->pose.orientation;

    // Get the yaw out of the quaternion, yay!
    // We can only do this because we know that the Quaternion will represent a
    // rotation in only one axis because the slam is 2D.
    // Otherwise things would be complicated
    double sign = (orientation.z > 0 ? 1 : -1);
    double yawR = 2*acos(orientation.w) * sign;
    double yawD = yawR * 180 / PI;

    globalBearingSLAM = yawR;
    // Flip because magic
    globalPositionSLAM.x = msg->pose.position.y;
    globalPositionSLAM.y = msg->pose.position.x;
    
    cout << "YAW: " << yawD << endl;
    cout << "(x, y) = (" << msg->pose.position.x << ", " << msg->pose.position.y << ")" << endl;
}

// PointGoal methods

bool PointGoal::isInsideObstacles(const nav_msgs::OccupancyGrid& map) const
{
    return (at(map, x, y) != 0);
}

bool PointGoal::isOutOfBounds(const nav_msgs::OccupancyGrid& map) const
{
    return (x >= map.info.width ||
            y >= map.info.height ||
            x < 0 ||
            y < 0);
}

void PointGoal::moveOutsideOfObstacles(const Point& currentPosition, const nav_msgs::OccupancyGrid& g)
{
    double deltaX = x - currentPosition.x;
    double deltaY = y - currentPosition.y;
    double distance = sqrt(deltaX*deltaX + deltaY*deltaY);

    for (double d = distance; d > 0; d -= 1.0)
    {
        double potentialX = currentPosition.x + deltaX * (d / distance);
        double potentialY = currentPosition.y + deltaY * (d / distance);

        if (at(g, potentialX, potentialY) == 0)
        {
            // Update myself and GTFO!
            x = potentialX;
            y = potentialY;
            return;
        }
    }

    // Ah! couldn't do anything! Goal will remain unchanged
}

string PointGoal::getGoalType() const
{
    return "Point";
}

double PointGoal::getHeuristicDistanceFrom(const P<int>& start) const
{
    // Return euclidean distance between start and this goal
    double dx = x - start.x;
    double dy = y - start.y;
    return sqrt(dx*dx + dy*dy);
}

bool PointGoal::contains(const P<int>& p) const
{
    return p.x == x && p.y == y;
}

PointGoal* PointGoal::clone() const
{
    return new PointGoal(*this);
}

// LineGoal methods

bool LineGoal::isInsideObstacles(const nav_msgs::OccupancyGrid& map) const
{
    // Lol, let's just hope we never have to deal with a situation where the entire line is
    // inside an obstacle (for now)
    // TODO: fixme
    return false;
}

bool LineGoal::isOutOfBounds(const nav_msgs::OccupancyGrid& map) const
{
    Pfloat start;
    start.x = x1;
    start.y = y1;

    Pfloat end;
    end.x = x2;
    end.y = y2;

    bool insideWindow = clipToWindow(start, end, map.info.width, map.info.height);
    return !insideWindow;
}

void LineGoal::moveOutsideOfObstacles(const Point& currentPosition, const nav_msgs::OccupancyGrid& g)
{
    // Lol, let's just hope we never have to deal with a situation where the entire line is
    // inside an obstacle (for now)
    // TODO: fixme
    return;
}

string LineGoal::getGoalType() const
{
    return "Line";
}

double LineGoal::getHeuristicDistanceFrom(const P<int>& sourcePoint) const
{
    Vec p1(x1, y1);
    Vec p2(x2, y2);
    Vec source(sourcePoint.x, sourcePoint.y);
    Vec line = p2 - p1;
    Vec toSource = source - p1;
    double projectionScalar = toSource.scalarProjectOnto(line);
    if (projectionScalar < 0)
    {
        return (source - p1).magnitude();
    }
    else if (projectionScalar > 1)
    {
        return (source - p2).magnitude();
    }
    else
    {
        Vec projected = line * projectionScalar;
        Vec perp = toSource - projected;
        return perp.magnitude();
    }
}

bool LineGoal::contains(const P<int>& p) const
{
    return getHeuristicDistanceFrom(p) <= 0.6; // or whatever
}

LineGoal* LineGoal::clone() const
{
    return new LineGoal(*this);
}

// http://stackoverflow.com/questions/9296059/read-pixel-value-in-bmp-file
// Call this function and pass in the filename of a bitmap you want to read.
// Will return a map of obstacles that is read from the bitmap.
// You must also pass in two points. This function will set them to whatever
// the current location and goal point are in the bitmap.
nav_msgs::OccupancyGrid readMapFromBitmap(const char* filename, Point& currentLocation, Point& goal1, Point& goal2, bool& foundGoal2)
// Obstacles should be red (RGB 255, 0, 0)
// Current location should be blue (RGB 0, 0, 255)
// Goal should be green (RGB 0, 255, 0)
{
    foundGoal2 = false;
    
    FILE* f = fopen(filename, "rb");
    if (!f)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, f);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));

    cout << "Reading a bitmap that is " << width << " by " << height << endl;

    // Creat the map
    nav_msgs::OccupancyGrid map;
    initializeOccupancyGrid(map, width, height, 0);

    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    for (int y = 0; y < height; y++)
    {
        // Read in a row at a time
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, f);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            // Extract R,G,B values from the pixel
            unsigned char blue = row[x*3];
            unsigned char green = row[x*3 + 1];
            unsigned char red = row[x*3 + 2];

            // Check if the pixel is an obstacle
            if (red == 255 && green == 0 && blue == 0)
            {
                at(map, x, y) = occupancyGridOccupied;
            }
            // Check if the pixel is the current location
            else if (blue == 255 && red == 0 && green == 0)
            {
                currentLocation.x = x;
                currentLocation.y = y;
            }
            // Check if the pixel is the goal1
            else if (green == 255 && red == 0 && blue == 0)
            {
                goal1.x = x;
                goal1.y = y;
            }
            // Check if the pixel is the goal2
            else if (green == 255 && red == 255 && blue == 0)
            {
                goal2.x = x;
                goal2.y = y;
                foundGoal2 = true;
            }
        }
    }

    fclose(f);

    return map;

}

// Draws the path that your function calculated onto the input image. Essentially a way
// to visualize the path. You must specify the input image to read, the output image,
// and the path. This function will essentially copy the input image onto the output
// image, but will superimpose the path on top in black.
void addPathToMap(const char* inputFile, const char* outputFile, Path& path, Goal& goal)
{
    // Open files
    FILE* in = fopen(inputFile, "rb");
    if (!in)
    {
        throw cantReadFile;
    }
    FILE* out = fopen(outputFile, "wb");
    if (!out)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, in);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));
    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    cout << "Adding a path to a bitmap that is " << width << " by " << height << endl;

    // Now let's create a temporary map, and use it to store which cells are in the path
    Map2D<bool> pathCells(width, height);
    for (int i = 0; i < path.size(); i++)
    {
        Point p = path[i];        
        if (p.x >= pathCells.getXSize() || p.y >= pathCells.getYSize())
        {
            cout << "Hmmm ... your path seems to be too big to fit onto this bitmap!" << endl;
        }
        pathCells.set(p.x, p.y, true);
    }

    // Copy the header verbatim into the output file
    fwrite(header, sizeof(unsigned char), 54, out);

    // Now iterate over the input image and copy it into the output image along with the path
    for (int y = 0; y < height; y++)
    {
        // Read a row from the input image
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, in);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            if (pathCells.get(x, y))
            {
                // If this pixel is in the path, set it to black
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
            // else if the pixel is white (background), let's recolour it!
            else if (row[x*3] == 255 && row[x*3 + 1] == 255 && row[x*3 + 2] == 255)
            {
                P<int> p(x, y);
                if (goal.contains(p))
                {
                    // If this pixel is in the goal, set it to purple
                    row[x*3] = 255; // blue
                    row[x*3 + 1] = 0; // green
                    row[x*3 + 2] = 255; // red
                }
                else
                {
                    // Else color this pixel some shade of cyan depending on how far away from goal
                    row[x*3] = 255; // blue
                    row[x*3 + 1] = min(255, (int)(goal.getHeuristicDistanceFrom(p) * 2)); // green
                    row[x*3 + 2] = 0; // red
                }
            }
        }

        // Spit the row back out to the output image
        fwrite(row, sizeof(unsigned char), bytesPerRow, out);
    }

    fclose(in);
    fclose(out);
}

int test_path_planner_main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "Oops, not right number args" << endl;
        exit(-1);
    }
    
    const char* input = argv[1];
    const char* output = argv[2];

    Point currentLocation;
    Point goal1;
    Point goal2;
    bool foundGoal2;
    nav_msgs::OccupancyGrid map = readMapFromBitmap(input, currentLocation, goal1, goal2, foundGoal2);

    Goal* goal = NULL;

    if (foundGoal2)
    {
        goal = new LineGoal(goal1.x, goal1.y, goal2.x, goal2.y);
    }
    else
    {
        goal = new PointGoal(goal1.x, goal1.y);
    }
    
    Path path = findPath(map, currentLocation, *goal);

    addPathToMap(input, output, path, *goal);

    return 0;
}

// Replace main with this function to interactively test the GPS conversion.
int test_convert_main(int argc, char** argv)
{
    while (!cin.eof())
    {
        int a, b, c;
        string d;

        cout << "Start: ";
        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        RawGpsInfo rawStart;
        rawStart.latDegrees = a;
        rawStart.latArcMinutes = b + (c / 60.0);
        rawStart.north = (d == "N");

        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        rawStart.lonDegrees = a;
        rawStart.lonArcMinutes = b + (c / 60.0);
        rawStart.east = (d == "E");

        cout << "End: ";
        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        RawGpsInfo rawEnd;
        rawEnd.latDegrees = a;
        rawEnd.latArcMinutes = b + (c / 60.0);
        rawEnd.north = (d == "N");

        cin >> a >> b >> c >> d;
        cout << "Saw: " << a  << ", " << b << ", " << c << ", " << d << endl;

        rawEnd.lonDegrees = a;
        rawEnd.lonArcMinutes = b + (c / 60.0);
        rawEnd.east = (d == "E");

        double metres, bearing;
        GpsInfo start = convertGpsInfoToRadians(rawStart);
        GpsInfo end = convertGpsInfoToRadians(rawEnd);
        gpsDelta(start, end, metres, bearing);

        cout << "Distance (m): " << metres << endl;
        cout << "Bearing: " << (int)(180 * bearing / PI) << " deg " << ((180 * bearing / PI) - (int)(180 * bearing / PI)) * 60 << " arcmin" << endl;
    }

    return 0;                
}

int ros_main(int argc, char** argv)
{
    // Setup this ros node
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    // intialize some SLAM stuff
    globalGoalPositionSLAM.x = 0;
    globalGoalPositionSLAM.y = 0;
    globalGoalPositionSLAM.z = 0;
    globalPositionSLAM.x = 0;
    globalPositionSLAM.y = 0;
    globalPositionSLAM.z = 0;

    // Subscribe to cam data
    // We only need the most recent one, right? So queue size of 1 is okay?
    // Switch which of these is commented out to change the method of passing
    // cam data to this module
    // ros::Subscriber camSub = n.subscribe("cam_data", 1, camCallbackLines);
    ros::Subscriber camSub = n.subscribe("art/map/local", 1, camCallbackOccupancyGrid);

    //Subscribe to GPS data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber gpsSub = n.subscribe("gps_data", 1, gpsCallback);

    //Subscribe to IMU data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber imuSub = n.subscribe("imu_data", 1, imuCallback);

    // Subscribe to Pose data from SLAM
    ros::Subscriber poseSub = n.subscribe("slam_out_pose", 1, poseCallback);

    // Initialize publisher to publish the path that we find
    pathPublisher = n.advertise<nav_msgs::Path>("path", 1000);

    // Initialize publisher to publish the map that we build
    mapPublisher = n.advertise<nav_msgs::OccupancyGrid>("art/map/localexpanded", 1000);

    // Initialize publishers to publish goal and current points
    currentPointPublisher = n.advertise<geometry_msgs::PointStamped>("currentPoint", 1000);
    goalPointPublisher = n.advertise<geometry_msgs::PointStamped>("goalPoint", 1000);

    // Initialize publihser to publish visuzlizations of line data from vision
    linesPublisher = n.advertise<visualization_msgs::Marker>("visionLines", 1000);

    // Initialize publisher to publish system commands (currently only "reset") for
    // the SLAM node
    slamCommandsPublisher = n.advertise<std_msgs::String>("syscommand", 1000);
    
    // Okay, this is a hack to test this module in the absence of a proper node
    // to produce real cam data.
    // To do this, what we do is we publish fake cam data for ourselves to 
    // consume.
    ros::Publisher camPub = n.advertise<geometry_msgs::Polygon>("cam_data", 10);

    // Make some fake cam data.
    // The format here is a little weird.
    // We're assuming the cam (after processing) gives us a set of line 
    // segments which describe the white lines we find.
    // So the format for the data is a vector of points, where each pair of
    // points is a line segment. So the first line segment goes from
    // vector[0] to vector[1]. The next is vector[2] to vector[3], etc.
    // Also, I included some pretty insanely large values in the path to test that
    // the clipping works as expected
    double path1[][2] = {{0,15.0},{23456.0,15.0} , {5.0,20.0},{20.0,20.0} , {-505.5,494.5},{555.0,-465.0} , {14.8,14.8},{28.5,10.0}};
    //double path2[][2] = {{0,150},{150,150} , {50,200},{200,200} , {195,195},{300,100} , {148,148},{285,100}};

    // Package up one of the paths we made into a polygon message so that we
    // can send it with ROS
    geometry_msgs::Polygon poly = makePolygon(path1);

    // Publish the path
    camPub.publish(poly);

    // Tell ROS to process any incoming messages (like the path we just published!)
    ros::spin();

    return 0;
}


int main(int argc, char** argv)
{
    //ros_main(argc, argv);
    test_path_planner_main(argc, argv);
    //test_convert_main(argc, argv);
}
