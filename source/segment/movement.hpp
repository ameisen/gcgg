#pragma once

#include "segment.hpp"

namespace gcgg::segments
{
  // Segment that has movement on X, Y, or Z
  class movement : public segment
  {
  protected:
    vector3<> start_position_;
    vector3<> end_position_;
    real feedrate_ = 0.0;
  public:
    movement(uint64 type) : segment(type) {}
    virtual ~movement() {}

    void set_positions(const vector3<> & __restrict start, const vector3<> & __restrict end) __restrict
    {
      start_position_ = start;
      end_position_ = end;
    }

    void set_feedrate(real feedrate) __restrict
    {
      feedrate_ = feedrate;
    }

    virtual std::string dump() const __restrict override
    {
      char buffer[512];
      std::string out;

      if (start_position_.x != end_position_.x)
      {
        sprintf(buffer, "%f", end_position_.x);
        out += " X";
        out += buffer;
      }

      if (start_position_.y != end_position_.y)
      {
        sprintf(buffer, "%f", end_position_.y);
        out += " Y";
        out += buffer;
      }

      if (start_position_.z != end_position_.z)
      {
        sprintf(buffer, "%f", end_position_.z);
        out += " Z";
        out += buffer;
      }

      sprintf(buffer, "%f", feedrate_);
      out += " F";
      out += buffer;

      return out;
    }

    real get_feedrate() const __restrict { return feedrate_; }
    const vector3<> & __restrict get_start_position() const __restrict { return start_position_; }
    const vector3<> & __restrict get_end_position() const __restrict { return end_position_; }
    virtual vector3<> get_vector() const __restrict override final { return end_position_ - start_position_; }

    void set_end_position(const vector3<> & __restrict position) __restrict
    {
      end_position_ = position;
    }

    void set_start_position(const vector3<> & __restrict position) __restrict
    {
      start_position_ = position;
    }

  public:
    // Lazy so making this public.
    // These are printer hints that are being kept around.
    real acceleration_hint_ = 0.0;
    vector3<> jerk_hint_;
    real jerk_extrude_hint_ = 0.0;
  };
}