/*

  scanline.c - grblHAL plugin for raster scanline moves with per-pixel laser power

  Copyright (c) 2026 Jondale Stratton

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if SCANLINE_ENABLE

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"
#include "grbl/planner.h"
#include "grbl/state_machine.h"

#include "scanline.h"

#ifndef SCANLINE_POOL_SIZE
#define SCANLINE_POOL_SIZE 32768
#endif

#ifndef SCANLINE_QUEUE_SIZE
#define SCANLINE_QUEUE_SIZE 8
#endif

#define SCANLINE_MCODE 230
#define SCANLINE_VERSION "0.01"
#define SCALE_ONE 32768
#define LEVEL_OFF 256
#define RATE_SHIFT 8
#define PWM_UNKNOWN UINT32_MAX

typedef enum {
    Slot_Free = 0,
    Slot_Filling,
    Slot_Queued,
    Slot_Active
} slot_state_t;

typedef struct scanline {
    volatile slot_state_t state;
    struct scanline *volatile next;
    spindle_ptrs_t spindle;
    uint32_t pixels;
    uint32_t lead_steps;
    uint32_t image_steps;
    uint32_t total_steps;
    axes_signals_t axis;
    uint8_t bits;
    uint8_t mask;
    bool rate_adjusted;
    uint32_t rate_factor;
    uint16_t lut[256];
    uint8_t *data;
    uint32_t size;
} scanline_t;

static struct {
    scanline_t *line;
    segment_t *segment;
    uint32_t step;
    uint32_t acc;
    uint32_t idx;
    uint32_t scale;
    uint32_t rate_factor;
    uint32_t want;
    uint32_t pwm;
    uint_fast16_t level;
} run;

static struct {
    float start[N_AXIS];
    float end[N_AXIS];
    float unit[N_AXIS];
    float lead;
    float length;
    float floor;
    uint32_t pixels;
    uint8_t bits;
    bool auto_lead;
    bool joined;
    bool more;
} job;

static struct {
    scanline_t *line;
    float end[N_AXIS];
    float unit[N_AXIS];
    bool active;
} chain;

static struct {
    scanline_t *line;
    uint32_t start;
    uint32_t len;
    bool error;
} fill;

static scanline_t slots[SCANLINE_QUEUE_SIZE];
static uint8_t pool[SCANLINE_POOL_SIZE];
static uint32_t pool_head = 0;
static spindle_ptrs_t *laser = NULL;
static spindle_param_t scan_param;

static user_mcode_ptrs_t user_mcode;
static stepper_pulse_start_ptr stepper_pulse_start;
static stepper_go_idle_ptr stepper_go_idle;
static on_report_options_ptr on_report_options;
static on_parser_init_ptr on_parser_init;
static on_reset_ptr on_reset;
static on_program_completed_ptr on_program_completed;
static float modal_floor = 0.0f;

static inline uint_fast8_t pixel_level (const scanline_t *line, uint32_t idx)
{
    uint32_t bit = idx * line->bits;

    return (line->data[bit >> 3] >> (8 - line->bits - (bit & 7))) & line->mask;
}

ISR_CODE static void ISR_FUNC(laser_write)(uint32_t pwm)
{
    run.pwm = pwm;
    laser->update_pwm(laser, pwm);
}

ISR_CODE static void ISR_FUNC(line_end)(void)
{
    if(run.pwm != laser->pwm_off_value)
        laser_write(laser->pwm_off_value);

    run.line->state = Slot_Free;
    run.line = NULL;
}

ISR_CODE static void ISR_FUNC(update_want)(void)
{
    int32_t off = (int32_t)laser->pwm_off_value;

    run.want = run.level != LEVEL_OFF
                ? (uint32_t)(off + ((((int32_t)run.line->lut[run.level] - off) * (int32_t)run.scale) / SCALE_ONE))
                : (uint32_t)off;
}

ISR_CODE static void ISR_FUNC(update_scale)(stepper_t *stepper)
{
    uint32_t scale = SCALE_ONE;

    run.segment = stepper->exec_segment;

    if(run.line->rate_adjusted) {
        uint64_t cycles = (uint64_t)run.segment->cycles_per_tick << (run.segment->amass_level + RATE_SHIFT);
        if(cycles > run.rate_factor)
            scale = (uint32_t)(((uint64_t)run.rate_factor * SCALE_ONE) / cycles);
    }

    if(laser->param->override_pct < 100)
        scale = scale * (uint32_t)laser->param->override_pct / 100;

    run.scale = scale;
}

ISR_CODE static scanline_t *ISR_FUNC(line_find)(const spindle_ptrs_t *spindle)
{
    uint_fast8_t idx = SCANLINE_QUEUE_SIZE;

    do {
        if(spindle == &slots[--idx].spindle)
            return &slots[idx];
    } while(idx);

    return NULL;
}

ISR_CODE static void ISR_FUNC(line_start)(scanline_t *line, stepper_t *stepper)
{
    if(run.line && run.line != line)
        line_end();

    if(line->state == Slot_Queued) {
        line->state = Slot_Active;
        run.line = line;
        run.step = run.acc = run.idx = 0;
        run.level = line->lead_steps == 0 ? pixel_level(line, 0) : LEVEL_OFF;
        run.rate_factor = line->rate_factor;
    }

    if(run.line) {
        update_scale(stepper);
        update_want();
        laser_write(run.want);
    }
}

ISR_CODE static void ISR_FUNC(scan_pulse_start)(stepper_t *stepper)
{
    if(stepper->new_block) {

        scanline_t *line = line_find(stepper->exec_block->spindle);

        if(line)
            line_start(line, stepper);
        else if(run.line)
            line_end();
    }

    if(run.line) {

        if(stepper->step_out.bits & run.line->axis.bits) {

            scanline_t *line = run.line;
            uint_fast16_t level = run.level;

            scanline_t *next;

            run.step++;

            if(run.step >= line->total_steps && (next = line->next) && next->state == Slot_Queued) {
                line->state = Slot_Free;
                next->state = Slot_Active;
                run.line = line = next;
                run.step = run.acc = run.idx = 0;
                run.rate_factor = line->rate_factor;
                level = pixel_level(line, 0);
            } else if(run.step < line->lead_steps || run.step >= line->lead_steps + line->image_steps)
                level = LEVEL_OFF;
            else if(run.step == line->lead_steps)
                level = pixel_level(line, 0);
            else if((run.acc += line->pixels) >= line->image_steps) {
                run.acc -= line->image_steps;
                level = pixel_level(line, ++run.idx);
            }

            if(level != run.level || run.segment != stepper->exec_segment) {
                run.level = level;
                if(run.segment != stepper->exec_segment)
                    update_scale(stepper);
                update_want();
            }

            if(run.want != run.pwm)
                laser_write(run.want);

            if(run.step >= line->total_steps)
                line_end();
        }
    }

    stepper_pulse_start(stepper);
}

ISR_CODE static void ISR_FUNC(scan_go_idle)(bool clear_signals)
{
    if(run.line) {
        laser_write(laser->pwm_off_value);
        run.pwm = PWM_UNKNOWN;
    }

    stepper_go_idle(clear_signals);
}

static void scan_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    laser->set_state(laser, state, rpm);
}

static spindle_state_t scan_get_state (spindle_ptrs_t *spindle)
{
    return laser->get_state(laser);
}

static uint_fast16_t scan_get_pwm (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->pwm_off_value;
}

ISR_CODE static void ISR_FUNC(scan_update_pwm)(spindle_ptrs_t *spindle, uint_fast16_t pwm)
{
    laser_write(pwm);
}

static void slots_reclaim (void)
{
    uint_fast8_t idx = SCANLINE_QUEUE_SIZE;

    run.line = NULL;

    do {
        if(slots[--idx].state != Slot_Filling)
            slots[idx].state = Slot_Free;
    } while(idx);
}

static void slots_release (void)
{
    if(fill.line)
        fill.line->state = Slot_Free;

    fill.line = NULL;
    fill.len = 0;
    fill.error = false;
    pool_head = 0;
    chain.active = false;

    slots_reclaim();
}

static bool scan_wait (void)
{
    // Lines still held with nothing planned belong to blocks that were flushed.
    if(plan_get_current_block() == NULL && state_get() == STATE_IDLE) {
        slots_reclaim();
        return true;
    }

    protocol_auto_cycle_start();

    return protocol_execute_realtime();
}

static scanline_t *slot_claim (void)
{
    do {
        uint_fast8_t idx = SCANLINE_QUEUE_SIZE;

        do {
            if(slots[--idx].state == Slot_Free) {
                slots[idx].state = Slot_Filling;
                return &slots[idx];
            }
        } while(idx);

    } while(scan_wait());

    return NULL;
}

static bool pool_region_free (uint32_t start, uint32_t end)
{
    uint_fast8_t idx = SCANLINE_QUEUE_SIZE;

    do {
        scanline_t *line = &slots[--idx];
        if(line->state >= Slot_Queued) {
            uint32_t line_start = (uint32_t)(line->data - pool);
            if(line_start < end && start < line_start + line->size)
                return false;
        }
    } while(idx);

    return true;
}

static status_code_t pool_reserve (uint32_t count)
{
    uint32_t need = fill.len + count;

    if(need > SCANLINE_POOL_SIZE)
        return Status_Overflow;

    do {
        if(fill.start + need <= SCANLINE_POOL_SIZE && pool_region_free(fill.start, fill.start + need))
            return Status_OK;

        if(fill.start && pool_region_free(0, need)) {
            memmove(pool, pool + fill.start, fill.len);
            fill.start = 0;
            return Status_OK;
        }
    } while(scan_wait());

    return Status_Unhandled;
}

static inline int_fast8_t hex_value (char c)
{
    if(c >= '0' && c <= '9')
        return c - '0';

    c |= 0x20;

    return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

static status_code_t scan_data (sys_state_t state, char *args)
{
    status_code_t status = Status_OK;
    uint32_t count;

    if(args == NULL || *args == '\0')
        return Status_InvalidStatement;

    bool store = state != STATE_CHECK_MODE;

    if(store && fill.line == NULL) {
        if((fill.line = slot_claim()) == NULL)
            return Status_Unhandled;
        fill.start = pool_head;
        fill.error = true;
    }

    if(store && fill.error) {
        fill.len = 0;
        fill.error = false;
    }

    count = strlen(args);

    if(count & 1)
        status = Status_BadNumberFormat;
    else if(store)
        status = pool_reserve(count >> 1);

    while(*args && status == Status_OK) {

        int_fast8_t hi = hex_value(args[0]), lo = hex_value(args[1]);

        if(hi < 0 || lo < 0)
            status = Status_BadNumberFormat;
        else if(store)
            pool[fill.start + fill.len++] = (uint8_t)((hi << 4) | lo);

        args += 2;
    }

    if(status != Status_OK && store)
        fill.error = true;

    return status;
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)SCANLINE_MCODE
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t scan_check (parser_block_t *gc_block)
{
    uint_fast8_t idx;
    float delta[N_AXIS], length = 0.0f, max_steps = 0.0f, lead_steps = 0.0f;
    float unit_scale = gc_block->modal.units_imperial ? MM_PER_INCH : 1.0f;
    bool has_axis[N_AXIS] = {0}, has_span[N_AXIS] = {0}, span, at_start = true;
    parser_state_t *gc_state = gc_get_state();

    if(!gc_state->spindle->hal->cap.laser)
        return Status_GcodeUnsupportedCommand;

    if(!(gc_block->words.p && gc_block->words.q))
        return Status_GcodeValueWordMissing;

    if(!(gc_block->words.x || gc_block->words.y || gc_block->words.z))
        return Status_GcodeNoAxisWords;

    if(!(isintf(gc_block->values.p) && isintf(gc_block->values.q)))
        return Status_GcodeCommandValueNotInteger;

    job.bits = (uint8_t)gc_block->values.q;

    if(gc_block->values.p < 1.0f || !(job.bits == 1 || job.bits == 2 || job.bits == 4 || job.bits == 8))
        return Status_GcodeValueOutOfRange;

    job.pixels = (uint32_t)gc_block->values.p;

    if((job.pixels * job.bits + 7) / 8 > SCANLINE_POOL_SIZE)
        return Status_GcodeValueOutOfRange;

    if(gc_block->words.e && !(gc_block->values.e == 0.0f || gc_block->values.e == 1.0f))
        return Status_GcodeValueOutOfRange;

    if(!gc_block->words.f && gc_state->feed_rate <= 0.0f)
        return Status_GcodeUndefinedFeedRate;

    if((gc_block->words.s && gc_block->values.s < 0.0f) || (gc_block->words.d && gc_block->values.d < 0.0f))
        return Status_NegativeValue;

    job.floor = gc_block->words.d ? gc_block->values.d : modal_floor;

    if(job.floor > (gc_block->words.s ? gc_block->values.s : gc_state->spindle->rpm))
        return Status_GcodeValueOutOfRange;

    has_axis[X_AXIS] = gc_block->words.x;
    has_axis[Y_AXIS] = gc_block->words.y;
    has_axis[Z_AXIS] = gc_block->words.z;

    has_span[X_AXIS] = gc_block->words.i;
    has_span[Y_AXIS] = gc_block->words.j;
    has_span[Z_AXIS] = gc_block->words.k;
    span = gc_block->words.i || gc_block->words.j || gc_block->words.k;

    for(idx = 0; idx < N_AXIS; idx++) {
        float position = gc_state->position[idx];
        if(has_axis[idx]) {
            float value = gc_block->values.xyz[idx] * unit_scale;
            job.end[idx] = gc_block->modal.distance_incremental ? position + value : value + gc_get_offset(idx, false);
        } else
            job.end[idx] = position;
        if(has_span[idx])
            job.start[idx] = job.end[idx] - gc_block->values.ijk[idx] * unit_scale;
        else
            job.start[idx] = span ? job.end[idx] : position;
        if(fabsf(job.start[idx] - position) * settings.axis[idx].steps_per_mm >= 0.5f)
            at_start = false;
    }

    if((job.joined = at_start && chain.active && !memcmp(gc_state->position, chain.end, sizeof(chain.end))))
        memcpy(job.start, gc_state->position, sizeof(job.start));

    for(idx = 0; idx < N_AXIS; idx++) {
        delta[idx] = job.end[idx] - job.start[idx];
        length += delta[idx] * delta[idx];
        max_steps = max(max_steps, fabsf(delta[idx]) * settings.axis[idx].steps_per_mm);
    }

    if((length = sqrtf(length)) == 0.0f)
        return Status_GcodeValueOutOfRange;

    if((float)job.pixels > max_steps)
        return Status_GcodeValueOutOfRange;

    for(idx = 0; idx < N_AXIS; idx++) {
        job.unit[idx] = delta[idx] / length;
        if(job.joined && fabsf(job.unit[idx] - chain.unit[idx]) > 0.0001f)
            return Status_GcodeValueOutOfRange;
    }

    job.length = length;
    job.more = gc_block->words.e && gc_block->values.e == 1.0f;
    job.auto_lead = !gc_block->words.r;
    job.lead = job.auto_lead ? 0.0f : gc_block->values.r * unit_scale;

    if(!job.auto_lead && !job.joined) {
        for(idx = 0; idx < N_AXIS; idx++)
            lead_steps = max(lead_steps, fabsf(job.unit[idx]) * job.lead * settings.axis[idx].steps_per_mm);
        if(job.lead < 0.0f || lead_steps < 1.0f)
            return Status_GcodeValueOutOfRange;
    }

    if(state_get() != STATE_CHECK_MODE &&
        !(fill.line && !fill.error && fill.len == (job.pixels * job.bits + 7) / 8))
        return Status_GcodeValueOutOfRange;

    // S is claimed so that the core does not synchronize the planner for a power change.
    gc_block->words.d = gc_block->words.e = gc_block->words.p = gc_block->words.q = gc_block->words.r = gc_block->words.s = Off;
    gc_block->words.x = gc_block->words.y = gc_block->words.z = Off;
    gc_block->words.i = gc_block->words.j = gc_block->words.k = Off;

    return Status_OK;
}

static status_code_t scan_validate (parser_block_t *gc_block)
{
    status_code_t status = scan_check(gc_block);

    if(status != Status_OK && fill.line) {
        fill.line->state = Slot_Free;
        fill.line = NULL;
    }

    return status;
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    return gc_block->user_mcode == (user_mcode_t)SCANLINE_MCODE
            ? scan_validate(gc_block)
            : (user_mcode.validate ? user_mcode.validate(gc_block) : Status_Unhandled);
}

// Distance to reach the scan speed from rest, plus one step segment, which can average across the end of the ramp.
static float auto_lead (float feed_rate)
{
    uint_fast8_t idx;
    float rate, max_rate = 0.0f, acceleration = 0.0f, lead, step = 0.0f;

    for(idx = 0; idx < N_AXIS; idx++) {
        float unit = fabsf(job.unit[idx]);
        if(unit > 0.0f) {
            float limit = settings.axis[idx].acceleration / unit;
            if(acceleration == 0.0f || limit < acceleration)
                acceleration = limit;
            limit = settings.axis[idx].max_rate / unit;
            if(max_rate == 0.0f || limit < max_rate)
                max_rate = limit;
            step = max(step, unit * settings.axis[idx].steps_per_mm);
        }
    }

    rate = min(feed_rate, max_rate);

    // An override below 100 % is ignored: lines queued under it stay long enough when it is raised again.
    if(sys.override.feed_rate > 100 && !sys.override.control.feed_rates_disable)
        rate = min(rate * 0.01f * (float)sys.override.feed_rate, max_rate);

    lead = rate * rate / (2.0f * acceleration) + rate / (60.0f * (float)ACCELERATION_TICKS_PER_SECOND);

    return ceilf(max(lead * step, 2.0f)) / step;
}

static void scan_execute (parser_block_t *gc_block)
{
    uint_fast8_t idx;
    uint_fast16_t level, levels;
    float power, lead_in, lead_out, travel, rate, pre[N_AXIS], end[N_AXIS];
    bool reposition = false;
    float spm;
    int32_t from, first;
    plan_line_data_t plan_data;
    plan_block_t *block;
    scanline_t *line = fill.line;
    parser_state_t *gc_state = gc_get_state();
    spindle_t *spindle = gc_state->spindle;

    if(line == NULL)
        return;

    fill.line = NULL;
    line->next = NULL;
    laser = spindle->hal;

    if(job.auto_lead)
        job.lead = auto_lead(gc_state->feed_rate);

    lead_in = job.joined ? 0.0f : job.lead;
    lead_out = job.more ? 0.0f : job.lead;
    travel = job.length + lead_in + lead_out;

    for(idx = 0; idx < N_AXIS; idx++) {
        pre[idx] = job.start[idx] - job.unit[idx] * lead_in;
        end[idx] = job.end[idx] + job.unit[idx] * lead_out;
    }

    memcpy(&scan_param, laser->param, sizeof(spindle_param_t));
    memcpy(&line->spindle, laser, sizeof(spindle_ptrs_t));
    line->spindle.param = &scan_param;
    line->spindle.set_state = scan_set_state;
    line->spindle.get_state = scan_get_state;
    line->spindle.get_pwm = scan_get_pwm;
    line->spindle.update_pwm = scan_update_pwm;

    line->data = pool + fill.start;
    line->size = fill.len;
    line->pixels = job.pixels;
    line->bits = job.bits;
    line->mask = (uint8_t)((1 << job.bits) - 1);
    line->rate_adjusted = spindle->state.on && spindle->state.ccw;

    power = gc_block->words.s ? gc_block->values.s : spindle->rpm;
    spindle->rpm = power;
    modal_floor = job.floor;
    levels = line->mask;
    for(level = 0; level <= levels; level++) {
        float value = job.floor + (power - job.floor) * (float)level / (float)levels;
        line->lut[level] = spindle->state.on && value > 0.0f ? (uint16_t)laser->get_pwm(laser, value) : (uint16_t)laser->pwm_off_value;
    }

    for(idx = 0; idx < N_AXIS; idx++) {
        if(lroundf(pre[idx] * settings.axis[idx].steps_per_mm) != lroundf(gc_state->position[idx] * settings.axis[idx].steps_per_mm))
            reposition = true;
    }

    if(reposition && !job.joined) {
        plan_data_init(&plan_data);
        plan_data.condition.rapid_motion = On;
        plan_data.condition.coolant = gc_state->modal.coolant;
        plan_data.line_number = gc_block->values.n;
        if(mc_line(pre, &plan_data) != Status_Handled) {
            line->state = Slot_Free;
            return;
        }
    }

    plan_data_init(&plan_data);
    plan_data.feed_rate = gc_state->feed_rate;
    plan_data.spindle.hal = &line->spindle;
    plan_data.spindle.state = spindle->state;
    plan_data.condition.coolant = gc_state->modal.coolant;
    plan_data.line_number = gc_block->values.n;

    if(mc_line(end, &plan_data) != Status_Handled ||
        (block = plan_get_recent_block()) == NULL || block->spindle.hal != &line->spindle) {
        line->state = Slot_Free;
        return;
    }

    line->axis.bits = 0;
    line->total_steps = block->step_event_count;
    for(idx = 0; idx < N_AXIS; idx++) {
        if(block->steps.value[idx] == block->step_event_count) {
            line->axis.bits = bit(idx);
            break;
        }
    }

    if(idx == N_AXIS)
        idx = 0;

    // Counted between the rounded step positions the planner uses, so the pixels cover exactly their own steps.
    spm = settings.axis[idx].steps_per_mm;
    from = lroundf((reposition && !job.joined ? pre : gc_state->position)[idx] * spm);
    first = lroundf(job.start[idx] * spm);
    line->lead_steps = (uint32_t)labs(first - from);
    line->image_steps = (uint32_t)labs(lroundf(job.end[idx] * spm) - first);
    rate = (float)hal.f_step_timer * 60.0f * travel / ((float)line->total_steps * block->programmed_rate) * (float)(1 << RATE_SHIFT);
    line->rate_factor = rate < 4294967040.0f ? (uint32_t)lroundf(rate) : UINT32_MAX;

    if(line->pixels > line->image_steps)
        line->pixels = line->image_steps;

    if((pool_head = fill.start + fill.len) >= SCANLINE_POOL_SIZE)
        pool_head = 0;

    line->state = Slot_Queued;

    if(job.joined && chain.line != line && chain.line->state != Slot_Free)
        chain.line->next = line;

    if((chain.active = job.more)) {
        chain.line = line;
        memcpy(chain.end, end, sizeof(end));
        memcpy(chain.unit, job.unit, sizeof(job.unit));
    }

    memcpy(gc_state->position, end, sizeof(end));
}

static void mcode_execute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == (user_mcode_t)SCANLINE_MCODE) {
        if(state != STATE_CHECK_MODE)
            scan_execute(gc_block);
    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void scan_program_completed (program_flow_t program_flow, bool check_mode)
{
    if(program_flow == ProgramFlow_CompletedM2 || program_flow == ProgramFlow_CompletedM30)
        modal_floor = 0.0f;

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void scan_parser_init (parser_state_t *gc_state)
{
    slots_release();
    modal_floor = 0.0f;

    if(on_parser_init)
        on_parser_init(gc_state);
}

ISR_CODE static void ISR_FUNC(scan_reset)(void)
{
    run.line = NULL;

    if(on_reset)
        on_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[SCANLINE:1,");
        hal.stream.write(uitoa(SCANLINE_POOL_SIZE));
        hal.stream.write("]" ASCII_EOL);
        report_plugin("Scanline", SCANLINE_VERSION);
    }
}

void scanline_init (void)
{
    static const sys_command_t command_list[] = {
        {"SLH", scan_data, {0}, { .str = "append hex pixel data for the next M230" } }
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    system_register_commands(&commands);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    stepper_pulse_start = hal.stepper.pulse_start;
    hal.stepper.pulse_start = scan_pulse_start;

    stepper_go_idle = hal.stepper.go_idle;
    hal.stepper.go_idle = scan_go_idle;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    on_parser_init = grbl.on_parser_init;
    grbl.on_parser_init = scan_parser_init;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = scan_program_completed;

    on_reset = grbl.on_reset;
    grbl.on_reset = scan_reset;
}

#endif
