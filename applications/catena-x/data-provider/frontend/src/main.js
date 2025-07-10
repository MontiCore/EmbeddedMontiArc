import { createApp } from 'vue'
import App from './App.vue'
import axios from 'axios'
import router from './router'
import store from './store'
import 'bootstrap/dist/css/bootstrap.min.css'

const app = createApp(App).use(store).use(router)
app.config.globalProperties.axios = axios
axios.defaults.baseURL = process.env.VUE_APP_BACKEND_URL
app.mount('#app')
