#include "common/GameState.h"
#include "common/MotionParameters.h"

const miam::RobotPosition COLLECT_ZONE_COORDS[9] =
{
    miam::RobotPosition(825, 1725, M_PI_2),
    miam::RobotPosition(75, 1325, M_PI),
    miam::RobotPosition(3000 - 75, 1325, M_PI),
    miam::RobotPosition(1100, 950, M_PI_2),
    miam::RobotPosition(3000 - 1100, 950, M_PI_2),
    miam::RobotPosition(75, 400, M_PI),
    miam::RobotPosition(3000 - 75, 400, M_PI),
    miam::RobotPosition(3000 - 775, 250, M_PI_2),
    miam::RobotPosition(775, 250, M_PI_2) // Small column action
};

const miam::RobotPosition CONSTRUCTION_ZONE_COORDS[4] =
{
    miam::RobotPosition(1200 + 50, 80, -M_PI_2),
    miam::RobotPosition(2900, 875, 0),
    miam::RobotPosition(750, 70, -M_PI_2),
    miam::RobotPosition(2700, 70, -M_PI_2)
};


void drawText(Cairo::RefPtr<Cairo::Context> const& cr, std::string const& text, double const& r = 0.0, double const& g = 0.0, double const& b=0.0)
{
    cr->set_font_size(100);
    cr->text_path(text);
    cr->set_line_width(25.0);
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->stroke_preserve();
    cr->set_line_width(10.0);
    cr->set_source_rgb(r, g, b);
    cr->stroke();
}

void drawZone(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& pos, bool const& isPlayingRightSide)
{
    cr->save();
    cr->translate((isPlayingRightSide ? 3000 - pos.x : pos.x), 2000. - pos.y);
    cr->rotate(-pos.theta);

    for (int j = 0; j < 4; j++)
    {
        cr->arc(0, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
        cr->fill();
    }
    cr->restore();
}

void GameState::draw(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& robotPosition, bool const& isPlayingRightSide)
{
    cr->set_source_rgb(1.0, 0.5, 0.0);

    for (int i = 0; i < 9; i++)
    {
        if (isCollectZoneFull[i])
            drawZone(cr, COLLECT_ZONE_COORDS[i], isPlayingRightSide);
    }

    // Add forbidden items
    cr->set_source_rgb(1.0, 0.0, 0.0);
    drawZone(cr, COLLECT_ZONE_COORDS[0], !isPlayingRightSide);

    cr->set_source_rgb(0.2, 1.0, 0.2);
    for (int i = 0; i < 4; i++)
    {
        if (isConstructionZoneUsed[i])
        {
            cr->save();
            cr->translate(CONSTRUCTION_ZONE_COORDS[i].x, 2000. - CONSTRUCTION_ZONE_COORDS[i].y);
            cr->rotate(-CONSTRUCTION_ZONE_COORDS[i].theta);

            for (int j = 0; j < 4; j++)
            {
                cr->arc(0, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
                cr->fill();
            }
            cr->restore();
        }
    }

    cr->set_source_rgb(1.0, 0.5, 0.0);
    cr->save();
    cr->translate(robotPosition.x, 2000. - robotPosition.y);
    cr->rotate(-robotPosition.theta);

    if (isFrontClawFull)
    {
        for (int j = 0; j < 4; j++)
        {
            cr->arc(182.0, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    if (isBackClawFull)
    {
        for (int j = 0; j < 4; j++)
        {
            cr->arc(-135.0, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    cr->restore();
}

// Path planning
#define GRID_RESOLUTION 25

void excludeRectangle(Map & map, int const lowerX, int const lowerY, int const upperX, int const upperY, int const margin = table_dimensions::table_margin, int const obstacleValue = 1)
{
    int const x = std::max(0, (lowerX - margin) / GRID_RESOLUTION);
    int const y = std::max(0, (lowerY - margin) / GRID_RESOLUTION);

    int const width = std::min(static_cast<int>(map.rows()) - x, std::min((lowerX - margin) / GRID_RESOLUTION, 0) + ((upperX - lowerX) + 2 * margin) / GRID_RESOLUTION);
    int const height = std::min(static_cast<int>(map.cols()) - y, std::min((lowerY - margin) / GRID_RESOLUTION, 0) + ((upperY - lowerY) + 2 * margin) / GRID_RESOLUTION);

    map.block(x, y, width, height).setConstant(obstacleValue);
}

Map GameState::generateMap()
{
    Map gameMap;

    int const borderMargin = static_cast<int>(std::ceil(table_dimensions::table_margin / GRID_RESOLUTION)); // Border margin, in grid unit.

    int const nRows = table_dimensions::table_size_x / GRID_RESOLUTION;
    int const nCols = table_dimensions::table_size_y / GRID_RESOLUTION;

    Map map(Eigen::MatrixXi::Zero(nRows, nCols), static_cast<double>(GRID_RESOLUTION));

    if (arePAMIMoving_)
    {
        // PAMI are optional exclude zone - put it first to be overwritten by other obstacles.
        excludeRectangle(map, 900, 1400, 3000, 2000, table_dimensions::table_margin, 2);
    }

    // Add obstacles linked to map: border
    map.bottomRows<borderMargin>().setConstant(1);
    map.topRows<borderMargin>().setConstant(1);
    map.leftCols<borderMargin>().setConstant(1);
    map.rightCols<borderMargin>().setConstant(1);

    // PAMI
    excludeRectangle(map, 0, 1800, 3000, 2000);
    // Scene
    excludeRectangle(map, 1050, 1550, 3000, 2000);


    // Other robot start / drop zones
    excludeRectangle(map, 0, 650, 450, 1100);
    excludeRectangle(map, 0, 0, 450, 150);
    excludeRectangle(map, 1500, 0, 2050, 550);
    excludeRectangle(map, 2000, 0, 2450, 150);

    for (int i = 0; i < 9; i++)
    {
        if (isCollectZoneFull[i])
        {
            RobotPosition bl = COLLECT_ZONE_COORDS[i] + RobotPosition(-200, -50).rotate(COLLECT_ZONE_COORDS[i].theta - M_PI_2);
            RobotPosition tr = COLLECT_ZONE_COORDS[i] + RobotPosition(200, 50).rotate(COLLECT_ZONE_COORDS[i].theta - M_PI_2);
            excludeRectangle(map, bl.x, bl.y, tr.x, tr.y, 150);
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if (isConstructionZoneUsed[i])
        {
            switch(i)
            {
                case 0: // (1200, 70, -M_PI_2)
                {
                    RobotPosition bl = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(-150,-50);
                    RobotPosition tr = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(100, -100);
                    excludeRectangle(map, bl.x, bl.y, tr.x, tr.y, 100);
                    break;
                }
                case 1: // (2900, 850, M_PI)
                {
                    RobotPosition bl = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(-250,-150);
                    RobotPosition tr = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(100,150);
                    excludeRectangle(map, bl.x, bl.y, tr.x, tr.y, 100);
                    break;
                }
                case 2: // (750, 70, -M_PI_2)
                {
                    RobotPosition bl = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(-150,-50);
                    RobotPosition tr = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(150, 150);
                    excludeRectangle(map, bl.x, bl.y, tr.x, tr.y, 100);
                    break;
                }
                case 3: // (2800, 70, -M_PI_2)
                {
                    RobotPosition bl = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(-150,-50);
                    RobotPosition tr = CONSTRUCTION_ZONE_COORDS[i] + RobotPosition(150,50);
                    excludeRectangle(map, bl.x, bl.y, tr.x, tr.y, 100);
                    break;
                }
            }
        }
    }

    return map;
}

void GameState::detectOtherRobotAction(std::vector<Obstacle> const& obstacles, double time, Logger *logger)
{
    for (int i = 0; i < 9; i++)
    {
        if (isCollectZoneFull[i])
        {
            for (auto const& o : obstacles)
            {
                if((std::get<0>(o) - COLLECT_ZONE_COORDS[i]).norm() < 300)
                {
                    if (timeOtherRobotEnteredZone_[i] < 0)
                        timeOtherRobotEnteredZone_[i] = time;
                    if (time - timeOtherRobotEnteredZone_[i] > 2.0)
                    {
                        *logger << "[GameState] Zone " << i << " picked up by other robot, removing" << std::endl;
                         isCollectZoneFull[i] = false;
                         break;
                    }
                }
                else
                {
                    timeOtherRobotEnteredZone_[i] = -1;
                }
            }
        }
    }
}
