import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    offers: [],
    datasets: []
  },
  getters: {
    getOffers (state) {
      return state.offers
    },
    getDatasets (state) {
      return state.datasets
    }
  },
  mutations: {
    setOffers (state, offers) {
      state.offers = offers
    },
    setDatasets (state, datasets) {
      state.datasets = datasets
    }
  },
  actions: {
    fetchOffers (context) {
      axios.get('/offers').then(response => {
        context.commit('setOffers', response.data)
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
