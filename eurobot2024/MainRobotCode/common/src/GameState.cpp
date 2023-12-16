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
        miam::RobotPosition(800, 1900),
        miam::RobotPosition(225, 1785),
        miam::RobotPosition(50, 1400),
        miam::RobotPosition(2785, 1000),
        miam::RobotPosition(2900, 600),
        miam::RobotPosition(225, 225)
};

void drawText(Cairo::RefPtr<Cairo::Context> const& cr, std::string const& text)
{
    cr->set_font_size(100);
    cr->text_path(text);
    cr->set_line_width(25.0);
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->stroke_preserve();
    cr->set_line_width(10.0);
    cr->set_source_rgb(0.0, 0.0, 0.0);
    cr->stroke();
}

void GameState::draw(Cairo::RefPtr<Cairo::Context> const& cr)
{
    cr->set_source_rgb(1.0, 0.0, 0.0);

    for (int i = 0; i < 6; i++)
    {
        cr->move_to(PLANT_COLLECT_COORD[i].x, 2000. - PLANT_COLLECT_COORD[i].y);
        drawText(cr, std::to_string(nPlantsPerZone[i]));
        cr->move_to(PLANT_DROP_DISPLAY_COORD[i].x, 2000. - PLANT_DROP_DISPLAY_COORD[i].y);
        drawText(cr, std::to_string(nPlantsCollected[i]));
    }


    cr->move_to(0, 100);
    drawText(cr, "In robot: " + std::to_string(nPlantsInRobot()));
}
