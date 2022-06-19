<template>
  <div class="left-area d-flex flex-column">
    <touch-slider vertical v-model:value="speedInput" :label="speedInput.toFixed(2)"/>
    <b-button class="tall-btn">BRAKE</b-button>
  </div>
  <div class="d-flex flex-column flex-grow-1 gap-2">
    <b-tabs class="main-area" content-class="mt-3">
      <b-tab title="Config" active>
        <div class="container">
          <div class="row">
            <p class="col">Speed limit:</p>
            <div class="col input-block">
              <b-form-input class="col" type="number"></b-form-input> <p>m/s</p>
              <b-form-input class="col" type="number"></b-form-input> <p>rpm</p>
            </div>
          </div>
          <div class="row">
            <p class="col">Steering limit:</p>
            <div class="col input-block">
              <b-form-input class="col" type="number"></b-form-input> <p>Radius</p>
              <b-form-input class="col" type="number"></b-form-input> <p>Curvature</p>
            </div>
          </div>
        </div>
      </b-tab>
      <b-tab title="Test">
        <b-button>{{ speedInput.toFixed(2) }}</b-button>
        <b-button>{{ steeringInput.toFixed(2) }}</b-button>
        <b-button @click="">TEST</b-button>
      </b-tab>
    </b-tabs>
    <div class="bottom-area d-flex">
      <touch-slider v-model:value="steeringInput" :label="steeringInput.toFixed(2)"/>
    </div>
  </div>
  <div class="right-area d-flex flex-column">
    <div class="flex-fill"></div>
    <b-button @click="toggleFullscreen"><i class="fas fa-expand"></i> FULLSCREEN</b-button>
    <b-button @click="">HARD RESET</b-button>
    <b-button @click="">SOFT RESET</b-button>
    <b-button variant="success" @click="">ENABLE</b-button>
    <b-button variant="danger" class="tall-btn"><i class="fas fa-ban"></i> STOP</b-button>
  </div>
</template>

<script>
import TouchSlider from './components/TouchSlider.vue'
import axios from 'axios';

export default {
  name: 'App',
  components: {
    TouchSlider,
  },
  data() {
    return {
      speedInput: 0,
      steeringInput: 0,
    }
  },
  methods: {
    toggleFullscreen() {
      if (!document.fullscreenElement) document.documentElement.requestFullscreen();
      else document.exitFullscreen();
    },
  }
}
</script>

<style lang="scss">
@import 'vars.scss';

* {
  user-select: none;
}

html,
body {
  width: 100%;
  height: 100%;
}

body {
  display: flex;
  justify-content: center;
  align-items: center;
  background: $dark  !important;
  color: $light !important;
}

#app {
  width: 100%;
  height: 100%;
  max-width: 60rem;
  max-height: 40rem;
  display: flex;
  justify-content: stretch;
  border-radius: 0.5rem;
  background: $gray-800;
  padding: 1rem;
  gap: 0.5rem;
}

.left-area {
  // width: 5rem;
  gap: 0.5rem;
}

.main-area {
  flex: 1;
  gap: 0.5rem;
  // background: $gray-700;
}

.bottom-area {
  // height: 5rem
  gap: 0.5rem;
}

.right-area {
  gap: 0.5rem;
}

.tall-btn {
  height: 5rem;
}

.input-block {
  display: flex;
  align-items: center;
  gap: 0.25rem;

  p {
    margin-bottom: 0;
  }
}
</style>
