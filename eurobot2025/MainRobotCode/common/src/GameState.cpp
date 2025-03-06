#include "common/GameState.h"

const miam::RobotPosition COLLECT_ZONE_COORDS[10] =
{
    miam::RobotPosition(1500, 1500),
    miam::RobotPosition(1000, 1300),
    miam::RobotPosition(2000, 1300),
    miam::RobotPosition(1000, 700),
    miam::RobotPosition(2000, 700),
    miam::RobotPosition(1500, 500),
    miam::RobotPosition(1500, 500),
    miam::RobotPosition(1500, 500),
    miam::RobotPosition(1500, 500),
    miam::RobotPosition(1500, 500)
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

void GameState::draw(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& robotPosition)
{
    cr->set_source_rgb(1.0, 0.0, 0.0);

    for (int i = 0; i < 10; i++)
    {
        if (isCollectZoneFull[i])
        {
            cr->save();
            cr->translate(COLLECT_ZONE_COORDS[i].x, 2000. - COLLECT_ZONE_COORDS[i].y);
            cr->rotate(-COLLECT_ZONE_COORDS[i].theta);

            for (int j = 0; j < 4; i++)
            {
                cr->arc(0, 50 * (-3 + 2 * i), 20, 0.0, 2.0 * M_PI);
                cr->fill();
            }
            cr->restore();
        }
    }


    cr->save();
    cr->translate(robotPosition.x, 2000. - robotPosition.y);
    cr->rotate(-robotPosition.theta);

    if (isFrontClawFull)
    {
        for (int j = 0; j < 4; j++)
        {
            cr->arc(50 * (-3 + 2 * j), 150, 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    if (isBackClawFull)
    {
        for (int j = 0; j < 4; j++)
        {
            cr->arc(50 * (-3 + 2 * j), -150, 20, 0.0, 2.0 * M_PI);
            cr->fill();
        }
    }
    cr->restore();

    // for (int i = 0; i < 4; i++)
    // {
    //     cr->move_to(POTS_DISPLAY_COORD[i].x, 2000. - POTS_DISPLAY_COORD[i].y);
    //     drawText(cr, std::to_string(nPotsInPile[i]), 0.5, 0.5, 0.5);
    // }
}
