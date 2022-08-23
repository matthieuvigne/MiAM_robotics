#ifndef APPLICATION_SCORING_HPP
#define APPLICATION_SCORING_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {

class ScoringWidget : public Gtk::Widget {

  public:
    ScoringWidget();
    virtual ~ScoringWidget();
    POINTER_TYPEDEF(ScoringWidget);
  
  public:
  
  private:

}; // class ScoringWidget
  
} // namespace application

#endif // APPLICATION_SCORING_HPP
