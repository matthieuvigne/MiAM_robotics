#include "common/GameState.h"

const miam::RobotPosition COLLECT_ZONE_COORDS[9] =
{
    miam::RobotPosition(825, 1725, M_PI_2),
    miam::RobotPosition(75, 1325, M_PI),
    miam::RobotPosition(3000 - 75, 1325, M_PI),
    miam::RobotPosition(1100, 950, M_PI_2),
    miam::RobotPosition(3000 - 1100, 950, M_PI_2),
    miam::RobotPosition(75, 400, M_PI),
    miam::RobotPosition(3000 - 75, 400, M_PI),
    miam::RobotPosition(775, 250, M_PI_2),
    miam::RobotPosition(3000 - 775, 250, M_PI_2)
};

const miam::RobotPosition CONSTRUCTION_ZONE_COORDS[4] =
{
    miam::RobotPosition(2800, 850, 0),
    miam::RobotPosition(750, 50, -M_PI_2),
    miam::RobotPosition(1200, 50, -M_PI_2),
    miam::RobotPosition(2800, 50, -M_PI_2)
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
            cr->arc(150, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    if (isBackClawFull)
    {
        for (int j = 0; j < 4; j++)
        {
            cr->arc(-150, 50 * (-3 + 2 * j), 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    cr->restore();
}
