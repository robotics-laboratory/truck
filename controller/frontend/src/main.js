import { createApp } from 'vue'
import App from './App.vue'

import BootstrapVue3 from 'bootstrap-vue-3'
import './style.scss'

import '@fortawesome/fontawesome-free/css/all.css'

const app = createApp(App)
app.use(BootstrapVue3)
app.mount('#app')
