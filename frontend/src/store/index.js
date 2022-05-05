import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    truckData: []
  },
  getters: {
    getTruckData (state) {
      return state.truckData
    },
    getDatasets (state) {
      return state.datasets
    }
  },
  mutations: {
    setTruckData (state, truckData) {
      state.truckData = truckData
    },
    setDatasets (state, datasets) {
      state.datasets = datasets
    }
  },
  actions: {
    fetchTruckData (context) {
      axios.get('/truckdata').then(response => {
        context.commit('setTruckData', response.data)
      })
    },
    fetchDatasets (context) {
      axios.get('/datasets').then(response => {
        context.commit('setDatasets', response.data)
      })
    }
  },
  modules: {
  }
})
