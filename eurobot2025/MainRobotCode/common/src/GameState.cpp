#include "common/GameState.h"

const miam::RobotPosition PLANT_COLLECT_COORD[6] =
{
    miam::RobotPosition(1500, 1500),
    miam::RobotPosition(1000, 1300),
    miam::RobotPosition(2000, 1300),
    miam::RobotPosition(1000, 700),
    miam::RobotPosition(2000, 700),
    miam::RobotPosition(1500, 500)
};

const miam::RobotPosition PLANT_DROP_DISPLAY_COORD[6] =
{
    miam::RobotPosition(225, 225),
    miam::RobotPosition(225, 1785),
    miam::RobotPosition(2785, 1000),
    miam::RobotPosition(2900, 600),
    miam::RobotPosition(50, 1400),
    miam::RobotPosition(800, 1900)
};


const miam::RobotPosition POTS_DISPLAY_COORD[4] =
{
    miam::RobotPosition(2700, 600),
    miam::RobotPosition(250, 1400),
    miam::RobotPosition(250, 600),
    miam::RobotPosition(2700, 1400),
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

    for (int i = 0; i < 6; i++)
    {
        cr->move_to(PLANT_COLLECT_COORD[i].x, 2000. - PLANT_COLLECT_COORD[i].y);
        drawText(cr, std::to_string(nPlantsPerZone[i]));
        cr->move_to(PLANT_DROP_DISPLAY_COORD[i].x, 2000. - PLANT_DROP_DISPLAY_COORD[i].y);
        drawText(cr, std::to_string(nPlantsCollected[i]));
    }

    for (int i = 0; i < 4; i++)
    {
        cr->move_to(POTS_DISPLAY_COORD[i].x, 2000. - POTS_DISPLAY_COORD[i].y);
        drawText(cr, std::to_string(nPotsInPile[i]), 0.5, 0.5, 0.5);
    }


    cr->move_to(0, 100);
    drawText(cr, "In robot: " + std::to_string(nPlantsInRobot()));

    cr->save();

    cr->translate(robotPosition.x, 2000. - robotPosition.y);
    // Minus sign: indirect convention is used in Cairo.
    cr->rotate(-robotPosition.theta);

    double const XPOS[6] = {150, 150, 150, -150, -150, -150};
    double const YPOS[6] = {-100, 0, 100, -100, 0, 100};
    for (int i = 0; i < 6; i++)
    {
        cr->arc(XPOS[i], YPOS[i], 20, 0.0, 2.0 * M_PI);
        switch(robotClawContent[i])
        {
            case ClawContent::EMPTY: cr->set_source_rgba(1.0, 1.0, 1.0, 0.0); break;
            case ClawContent::UNKNOWN_PLANT: cr->set_source_rgb(1.0, 1.0, 0.0); break;
            default: cr->set_source_rgb(1.0, 1.0, 1.0); break;
        }
        cr->fill();
    }
    cr->restore();
}
