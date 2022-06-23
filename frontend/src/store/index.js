import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    truckData: [],
    offers: {},
    datarows: []
  },
  getters: {
    getTruckData (state) {
      return state.truckData
    },
    getOffers (state) {
      return state.offers
    },
    getDatarows (state) {
      return state.datarows
    }
  },
  mutations: {
    setTruckData (state, truckData) {
      state.truckData = truckData
    },
    setOffers (state, offers) {
      state.offers = offers
    },
    setDatarows (state, datarows) {
      state.datarows = datarows
    }
  },
  actions: {
    fetchTruckData (context) {
      axios.get('/truckdata').then(response => {
        context.commit('setTruckData', response.data)
      })
    },
    fetchOffers (context) {
      axios.get('/offers').then(response => {
        context.commit('setOffers', response.data)
      })
    },
    async deleteOffer (context, offerId) {
      return axios.delete(`/offers/${offerId}`)
    },
    async uploadCsv (context, file) {
      const formData = new FormData()
      formData.append('file', file)
      return axios.post('/fileupload', formData).then((response) => {
        context.commit('setDatarows', response.data)
      })
    }
  },
  modules: {
  }
})
