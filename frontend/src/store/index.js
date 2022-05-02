import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    truckData: []
  },
  getters: {
    getTruckData (state) {
      return state.truckData
    }
  },
  mutations: {
    setTruckData (state, truckData) {
      state.truckData = truckData
    }
  },
  actions: {
    fetchTruckData (context) {
      axios.get('/truckdata').then(response => {
        context.commit('setTruckData', response.data)
      })
    }
  },
  modules: {
  }
})
