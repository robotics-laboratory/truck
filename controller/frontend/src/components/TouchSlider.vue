<template>
  <div class="touch-slider" :class="{ 'vertical': vertical, 'horizontal': !vertical }" @mousedown="mouseDown = true">
    <div class="track">
      <div class="track-bar" :style="progressStyle"></div>
    </div>
    <div class="event-target" ref="target"></div>
    <div class="thumb" :style="thumbStyle">
      <i class="fas fa-house" v-if="atZero"></i>
      <span v-else>{{ label }}</span>
    </div>
  </div>
</template>

<script>
import _ from 'lodash';

const ZERO_EPS = 0.05;
const VIBRATE_MS = 30;
const VIBRATE_THROTTLE = 100;

export default {
  name: 'TouchSlider',
  props: {
    value: { type: Number, default: 0 },
    step: { type: Number, default: 0.01 },
    vertical: { type: Boolean, default: false },
    label: { type: String, default: "" },
  },
  data() {
    return {
      mouseDown: false,
      mounted: false,
    }
  },
  mounted() {
    window.addEventListener('mousemove', this.mousemove.bind(this), { passive: false });
    window.addEventListener('mouseup', () => { this.mouseDown = false }, { passive: false });
    window.addEventListener('touchstart', this.touchmove.bind(this), { passive: false });
    window.addEventListener('touchmove', this.touchmove.bind(this), { passive: false });
    window.addEventListener('resize', () => {
      let value = this.value;
      this.$emit('update:value', 0);
      this.$nextTick(() => this.$emit('update:value', value))
    });
    this.mounted = true;
  },
  computed: {
    thumbStyle() {
      if (!this.mounted) return "";
      let [range, offset] = this.getDimensions();
      offset = (this.value + 1) * range;
      if (this.vertical) return `bottom: ${offset}px`;
      else return `left: ${offset}px`;
    },
    progressStyle() {
      if (!this.mounted) return "";
      let [range, offset] = this.getDimensions();
      let size = Math.round(Math.abs(this.value) * range);
      let style = "";
      if (this.vertical) {
        style += `height: ${size}px; `;
        style += this.value > 0 ? "bottom: 50%" : "top: 50%";
      } else {
        style += `width: ${size}px; `;
        style += this.value > 0 ? "left: 50%" : "right: 50%";
      }
      return style;
    },
    atZero() {
      return Math.abs(this.value) < ZERO_EPS;
    }
  },
  methods: {
    touchmove(e) {
      let rect = this.$el.getBoundingClientRect();
      let x, y, valid = false;
      for (let touch of e.touches) {
        x = touch.pageX;
        y = touch.pageY;
        if (x < rect.x || x > rect.x + rect.width) continue;
        if (y < rect.y || y > rect.y + rect.height) continue;
        valid = true;
        e.preventDefault();
        break;
      }
      if (valid) this.updateValue(this.vertical ? y : x);
    },
    mousemove(e) {
      e.preventDefault();
      if (!this.mouseDown) return;
      this.updateValue(this.vertical ? e.pageY : e.pageX);
    },
    getDimensions() {
      let rect = this.$refs.target.getBoundingClientRect();
      if (this.vertical) return [rect.height / 2, rect.y];
      else return [rect.width / 2, rect.x];
    },
    updateValue(position) {
      let [range, offset] = this.getDimensions();
      let value = Math.max(-1, Math.min(1, (position - offset) / range - 1));
      if (this.vertical) value = -value;
      value = Math.ceil(value / this.step) * this.step;
      if (Math.abs(value) < ZERO_EPS) {
        if (Math.abs(this.value) >= ZERO_EPS) {
          this.vibrate();
        }
        value = 0;
      }
      this.$emit('update:value', value);
    },
    vibrate: _.throttle(() => {
      if (navigator.vibrate != undefined) navigator.vibrate(VIBRATE_MS);
    }, VIBRATE_THROTTLE),
  }
}
</script>

<style lang="scss">
@import '../vars.scss';

$slider-size: 4rem;
$touch-padding: 1rem;
$track-size: 2rem;
$thumb-size: 4rem;

.touch-slider {
  display: block;
  width: 100%;
  height: 100%;
  position: relative;
}

.track {
  position: absolute;
  border-radius: $track-size;
  background: $gray-500;
  overflow: hidden;
}

.track-bar {
  background: $danger;
  width: 100%;
  height: 100%;
  position: absolute;
}

.progress {
  position: absolute;
}

.event-target {
  // background: transparentize(white, 0.8);
  width: 100%;
  height: 100%;
}

.thumb {
  position: absolute;
  background: $light;
  // border: 0.4rem solid $gray-500;
  height: $thumb-size;
  width: $thumb-size;
  margin: calc($touch-padding - $thumb-size / 2);
  border-radius: 100%;
  display: flex;
  justify-content: center;
  align-items: center;

  i {
    font-size: 1.5rem;
    color: $danger;
  }
}

.touch-slider.vertical {
  // background: transparentize(red, 0.8);
  width: $slider-size;
  height: 100%;
  padding-top: $touch-padding;
  padding-bottom: $touch-padding;

  .track {
    height: calc(100% - $touch-padding * 2);
    width: $track-size;
    left: 50%;
    transform: translateX(-50%);
  }

  .thumb {
    left: 50%;
    transform: translateX(-50%);
    margin-left: 0;
    margin-right: 0;
  }
}

.touch-slider.horizontal {
  // background: transparentize(blue, 0.8);
  height: $slider-size;
  width: 100%;
  padding-left: $touch-padding;
  padding-right: $touch-padding;

  .track {
    width: calc(100% - $touch-padding * 2);
    height: $track-size;
    top: 50%;
    transform: translateY(-50%);
  }

  .thumb {
    top: 50%;
    transform: translateY(-50%);
    margin-top: 0;
    margin-bottom: 0;
  }
}
</style>
