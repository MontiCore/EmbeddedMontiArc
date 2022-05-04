import { createApp } from 'vue'
import App from './App.vue'
import axios from 'axios'
import router from './router'
import store from './store'
import 'bootstrap/dist/css/bootstrap.min.css'
import 'bootstrap/dist/js/bootstrap.js'

const app = createApp(App).use(store).use(router)
app.config.globalProperties.axios = axios
axios.defaults.baseURL = 'http://localhost:8080'
app.mount('#app')
