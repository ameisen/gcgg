#include "gcgg.hpp"

#include "segment.hpp"
#include "arc.hpp"
#include "extrusion.hpp"
#include "extrusion_move.hpp"
#include "hop.hpp"
#include "linear.hpp"
#include "travel.hpp"

void segments::segment::calculate_motion_data()
{
  motion_data_.calculated_ = true;

  bool prev_motion = false;
  bool next_motion = false;

  const auto is_motion = [](const segment * __restrict seg) -> bool
  {
    switch (seg->get_type())
    {
    case segments::arc::type:
    case segments::extrusion_move::type:
    case segments::hop::type:
    case segments::linear::type:
    case segments::travel::type:
      return true;
    default:
      return false;
    }
  };

  // TODO if we are in a pure extrusion command, we can interleave motion _iif_ the next motion is a travel.
  if (!is_motion(this))
  {
    // If this isn't a true motion, there's nothing to calculate.
    return;
  }

  segments::movement * move = static_cast<segments::movement *>(this);

  motion_data_.plateau_feedrate_ = move->get_feedrate();

  if (prev_segment_ && is_motion(prev_segment_))
  {
    if (prev_segment_->get_type() == segments::arc::type)
    {
      const segments::arc * arc_move = static_cast<const segments::arc *>(prev_segment_);
      motion_data_.entry_feedrate_ = arc_move->get_feedrate_element(1);
    }
    else
    {
      const segments::movement * seg_move = static_cast<const segments::movement *>(prev_segment_);
      motion_data_.entry_feedrate_ = seg_move->get_feedrate();
    }
  }
  if (next_segment_ && is_motion(next_segment_))
  {
    if (next_segment_->get_type() == segments::arc::type)
    {
      const segments::arc * arc_move = static_cast<const segments::arc *>(next_segment_);
      motion_data_.exit_feedrate_ = arc_move->get_feedrate_element(0);
    }
    else
    {
      const segments::movement * seg_move = static_cast<const segments::movement *>(next_segment_);
      motion_data_.exit_feedrate_ = seg_move->get_feedrate();
    }
  }
}
