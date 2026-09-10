/**
 * Deterministic single-car elevator controller; parity with elevator.py.
 *
 * new ElevatorController(floorHeights, {initialFloor=0, maxSpeed=1.8,
 *   acceleration=1.0, doorSeconds=1.8, dwellSeconds=3.0})
 * All distances are metres, all durations seconds, and floors zero-based.
 * requestFloor(floor), requestHall(floor, 'up'|'down') register unique calls.
 * Calls to a landing are grouped FIFO. All its cabin/hall requests clear only
 * when its doors fully open (both hall directions count as served).
 * openDoors()/closeDoors() report whether the action is safe now. setObstruction
 * reopens/holds doors only at landings. pressAlarm() signals without stopping.
 * tick(dt) advances simulation; snapshot() returns an independent plain object.
 *
 * Snapshot: state ('idle'|'closing'|'moving'|'opening'|'dwell'), floor (last
 * reached landing), targetFloor, position (cabin floor Z), velocity, doorOpen
 * (0..1), cabinLights [boolean], hallLights [{up,down}], queue [floor indices],
 * obstruction, openButtonLit, closeButtonLit, alarmLit. No light is inferred
 * from a selected floor or camera position. No renderer or physics dependency.
 */

export class ElevatorController {
  constructor(floorHeights, options = {}) {
    this.floorHeights = [...floorHeights].map(v => this._number(v, 'floor height'));
    if (this.floorHeights.length < 2) throw new RangeError('at least two floor heights are required');
    if (this.floorHeights.some((v, i, a) => i > 0 && v <= a[i - 1])) {
      throw new RangeError('floor heights must be strictly increasing');
    }
    this.maxSpeed = this._positive(options.maxSpeed ?? 1.8, 'maxSpeed');
    this.acceleration = this._positive(options.acceleration ?? 1.0, 'acceleration');
    this.doorSeconds = this._positive(options.doorSeconds ?? 1.8, 'doorSeconds');
    this.dwellSeconds = this._number(options.dwellSeconds ?? 3.0, 'dwellSeconds');
    if (this.dwellSeconds < 0) throw new RangeError('dwellSeconds must be nonnegative');
    this.floor = options.initialFloor ?? 0;
    this._validateFloor(this.floor);
    this.position = this.floorHeights[this.floor];
    this.velocity = 0;
    this.doorOpen = 0;
    this.state = 'idle';
    this.targetFloor = null;
    this.obstruction = false;
    this._cabin = this.floorHeights.map(() => false);
    this._hall = this.floorHeights.map(() => ({up: false, down: false}));
    this._queue = [];
    this._dwellRemaining = 0;
    this._motion = null;
    this._buttonSeconds = {open: 0, close: 0, alarm: 0};
  }

  _number(value, name) {
    if (typeof value !== 'number' || !Number.isFinite(value)) throw new RangeError(`${name} must be a finite number`);
    return value;
  }

  _positive(value, name) {
    this._number(value, name);
    if (value <= 0) throw new RangeError(`${name} must be positive`);
    return value;
  }

  _validateFloor(floor) {
    if (!Number.isInteger(floor) || floor < 0 || floor >= this.floorHeights.length) {
      throw new RangeError('floor must be a valid zero-based integer index');
    }
  }

  requestFloor(floor) {
    this._validateFloor(floor);
    if (this._cabin[floor]) return false;
    this._cabin[floor] = true;
    this._enqueue(floor);
    return true;
  }

  requestHall(floor, direction) {
    this._validateFloor(floor);
    if (direction !== 'up' && direction !== 'down') throw new RangeError("direction must be 'up' or 'down'");
    if ((floor === 0 && direction === 'down') || (floor === this.floorHeights.length - 1 && direction === 'up')) {
      throw new RangeError('hall direction leads beyond the building');
    }
    if (this._hall[floor][direction]) return false;
    this._hall[floor][direction] = true;
    this._enqueue(floor);
    return true;
  }

  _enqueue(floor) {
    if (!this._queue.includes(floor)) this._queue.push(floor);
    if (this.state !== 'moving' && floor === this.floor) {
      if (this.state === 'dwell') {
        this._serveLanding();
        this._dwellRemaining = this.dwellSeconds;
      } else this._beginOpening();
    } else if (this.state === 'idle') this._dispatch();
  }

  openDoors() {
    this._buttonSeconds.open = 0.8;
    if (this.state === 'moving') return false;
    if (this.state === 'dwell') this._dwellRemaining = this.dwellSeconds;
    else this._beginOpening();
    return true;
  }

  closeDoors() {
    this._buttonSeconds.close = 0.8;
    if (this.state === 'moving' || this.obstruction) return false;
    if (this.doorOpen > 0 || this.state === 'opening') this._beginClosing();
    else if (this.state === 'idle') this._dispatch();
    return true;
  }

  pressAlarm() {
    this._buttonSeconds.alarm = 1.5;
    return true;
  }

  setObstruction(obstructed) {
    if (typeof obstructed !== 'boolean') throw new TypeError('obstructed must be a boolean');
    this.obstruction = obstructed;
    if (obstructed && this.state === 'closing') this._beginOpening();
    else if (!obstructed && this.state === 'idle') this._dispatch();
  }

  _beginOpening() {
    this.state = 'opening';
    this.targetFloor = this.floor;
    this.velocity = 0;
  }

  _beginClosing() {
    this.state = 'closing';
    this.targetFloor = this._queue.length ? this._queue[0] : null;
  }

  _serveLanding() {
    this._cabin[this.floor] = false;
    this._hall[this.floor] = {up: false, down: false};
    this._queue = this._queue.filter(f => f !== this.floor);
  }

  _dispatch() {
    if (this.doorOpen !== 0) throw new Error('travel interlock: cabin doors must be fully closed');
    if (!this._queue.length) {
      this.state = 'idle';
      this.targetFloor = null;
      return;
    }
    this.targetFloor = this._queue[0];
    if (this.targetFloor === this.floor || this.obstruction) this._beginOpening();
    else this._beginMovement();
  }

  _beginMovement() {
    if (this.doorOpen !== 0 || this.obstruction) throw new Error('travel interlock: doors must be closed and clear');
    const distance = Math.abs(this.floorHeights[this.targetFloor] - this.position);
    const ramp = Math.min(this.maxSpeed / this.acceleration, Math.sqrt(distance / this.acceleration));
    const peak = this.acceleration * ramp;
    const cruise = Math.max(0, (distance - this.acceleration * ramp * ramp) / peak);
    this._motion = {
      start: this.position,
      sign: this.floorHeights[this.targetFloor] > this.position ? 1 : -1,
      distance, ramp, peak, cruise,
      duration: 2 * ramp + cruise,
      elapsed: 0,
    };
    this.state = 'moving';
  }

  tick(dt) {
    let remaining = this._number(dt, 'dt');
    if (remaining < 0) throw new RangeError('dt must be nonnegative');
    for (const button of Object.keys(this._buttonSeconds)) {
      this._buttonSeconds[button] = Math.max(0, this._buttonSeconds[button] - remaining);
    }
    while (remaining > 0) {
      if (this.state === 'idle') break;
      if (this.state === 'moving') {
        const motion = this._motion;
        const available = motion.duration - motion.elapsed;
        const step = Math.min(remaining, available);
        motion.elapsed += step;
        remaining = Math.max(0, remaining - step);
        const t = motion.elapsed, ramp = motion.ramp, peak = motion.peak;
        let displacement, speed;
        if (t < ramp) {
          displacement = 0.5 * this.acceleration * t * t;
          speed = this.acceleration * t;
        } else if (t < ramp + motion.cruise) {
          displacement = 0.5 * this.acceleration * ramp * ramp + peak * (t - ramp);
          speed = peak;
        } else {
          const untilArrival = Math.max(0, motion.duration - t);
          displacement = motion.distance - 0.5 * this.acceleration * untilArrival * untilArrival;
          speed = this.acceleration * untilArrival;
        }
        this.position = motion.start + motion.sign * displacement;
        this.velocity = motion.sign * speed;
        if (step >= available) {
          this.floor = this.targetFloor;
          this.position = this.floorHeights[this.floor];
          this.velocity = 0;
          this._motion = null;
          this._beginOpening();
        }
      } else if (this.state === 'opening') {
        const available = (1 - this.doorOpen) * this.doorSeconds;
        const step = Math.min(remaining, available);
        this.doorOpen = Math.min(1, this.doorOpen + step / this.doorSeconds);
        remaining = Math.max(0, remaining - step);
        if (step >= available) {
          this.doorOpen = 1;
          this._serveLanding();
          this.targetFloor = null;
          this.state = 'dwell';
          this._dwellRemaining = this.dwellSeconds;
        }
      } else if (this.state === 'dwell') {
        if (this.obstruction) {
          this._dwellRemaining = this.dwellSeconds;
          break;
        }
        const step = Math.min(remaining, this._dwellRemaining);
        this._dwellRemaining = Math.max(0, this._dwellRemaining - step);
        remaining = Math.max(0, remaining - step);
        if (this._dwellRemaining === 0) this._beginClosing();
      } else if (this.state === 'closing') {
        if (this.obstruction) {
          this._beginOpening();
          continue;
        }
        const available = this.doorOpen * this.doorSeconds;
        const step = Math.min(remaining, available);
        this.doorOpen = Math.max(0, this.doorOpen - step / this.doorSeconds);
        remaining = Math.max(0, remaining - step);
        if (step >= available) {
          this.doorOpen = 0;
          this._dispatch();
        }
      } else throw new Error(`unknown elevator state: ${this.state}`);
    }
    return this.snapshot();
  }

  snapshot() {
    return {
      state: this.state,
      floor: this.floor,
      targetFloor: this.targetFloor,
      position: this.position,
      velocity: this.velocity,
      doorOpen: this.doorOpen,
      cabinLights: [...this._cabin],
      hallLights: this._hall.map(calls => ({...calls})),
      queue: [...this._queue],
      obstruction: this.obstruction,
      openButtonLit: this._buttonSeconds.open > 0,
      closeButtonLit: this._buttonSeconds.close > 0,
      alarmLit: this._buttonSeconds.alarm > 0,
    };
  }
}
