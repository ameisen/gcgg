#pragma once

#include "command.hpp"

namespace gcgg::segments
{
  class segment : public command
  {
  public:
    segment(uint64 type) : command(type) {}
    virtual ~segment() {}

    virtual vector3<> get_vector() const __restrict { return vector3<>::zero; }

    virtual bool is_segment() const __restrict override final { return true; }
    virtual bool is_instruction() const __restrict override final { return false; }

  public: // make private when I feel like it.
    bool from_arc_ = false;
    segment * prev_segment_ = nullptr;
    segment * next_segment_ = nullptr;

    // Calculated motion data. Can be used for internal processing, emitting gcode2/gg, so forth.
    struct motion_data
    {
      bool calculated_ = false;
      real entry_feedrate_ = 0.0;
      real plateau_feedrate_ = 0.0;
      real exit_feedrate_ = 0.0;
    } motion_data_;

    bool motion_data_ready() const __restrict
    {
      return motion_data_.calculated_;
    }

    virtual vector3<> get_velocity() const __restrict { return vector3<>::zero; }
  };
}
