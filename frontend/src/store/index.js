import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    offers: []
  },
  getters: {
    getOffers (state) {
      return state.offers
    }
  },
  mutations: {
    setOffers (state, offers) {
      state.offers = offers
    }
  },
  actions: {
    fetchOffers (context) {
      axios.get('/datasets').then(response => {
        context.commit('setOffers', response.data)
      })
    }
  },
  modules: {
  }
})
