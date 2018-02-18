#pragma once

#include "output/state.hpp"
#include "config.hpp"

namespace gcgg
{
  class command
  {
  protected:
    // The below allow the two instruction streams (motion and non-motion) to be executed simultaneously.
    // This also allows the streams to have no particular references to one another, as we know when building
    // the commands and these flags their relation to one another.
    // Some commands can force a delay (like set temperature and wait) which would stop motion commands.
    // This is important though as it allows motion to continue without a hitch even when commands like setting
    // temperature exist between two moves, as the motion system will continue operating,
    // and the temperature or other command will execute when the motion system returns from the interrupt.

    // Should the next motion command be immediately executed after this, or do we stop?
    bool execute_motion_ = true;
    // Should we begin executing the non-motion instruction queue after this motion completes?
    bool execute_instruction_ = false;

    uint64 type_;
    bool delay_;
  public:
    command(uint64 type, bool delay = false) : type_(type), delay_(delay) {}
    virtual ~command() {}

    virtual std::string dump() const __restrict = 0;

    void execute_motion_after(bool val) __restrict
    {
      execute_motion_ = val;
    }

    void execute_instruction_after(bool val) __restrict
    {
      execute_instruction_ = val;
    }

    uint64 get_type() const __restrict
    {
      return type_;
    }

    bool is_delay() const __restrict
    {
      return delay_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict = 0;

    virtual bool is_segment() const __restrict = 0;
    virtual bool is_instruction() const __restrict = 0;

    virtual void compute_motion(const config & __restrict cfg, bool require_jerk) __restrict {}
  };
}
