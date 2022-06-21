import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    offers: [],
    boughtOffers: [],
    datasets: [],
    currentDataset: null
  },
  getters: {
    getOffers (state) {
      return state.offers
    },
    getBoughtOffers (state) {
      return state.boughtOffers
    },
    getDatasets (state) {
      return state.datasets
    },
    getCurrentDataset (state) {
      return state.currentDataset
    }
  },
  mutations: {
    setOffers (state, offers) {
      state.offers = offers
    },
    setBoughtOffers (state, offers) {
      state.boughtOffers = offers
    },
    setDatasets (state, datasets) {
      state.datasets = datasets
    },
    setCurrentDataset (state, dataset) {
      state.currentDataset = dataset
    }
  },
  actions: {
    fetchOffers (context) {
      axios.get('/offers').then(response => {
        context.commit('setOffers', response.data)
      })
    },
    fetchBoughtOffers (context) {
      axios.get('/offers', { params: { bought: true } }).then(response => {
        context.commit('setBoughtOffers', response.data)
      })
    },
    fetchDatasets (context) {
      axios.get('/datasets').then(response => {
        context.commit('setDatasets', response.data)
      })
    },
    async buyOffer (context, offerId) {
      return axios.post('offers/' + offerId)
    },
    async fetchDataset (context, id) {
      return axios.get(`datasets/${id}`)
    },
    setCurrentDataset (context, dataset) {
      context.commit('setCurrentDataset', dataset)
    }
  },
  modules: {
  }
})
