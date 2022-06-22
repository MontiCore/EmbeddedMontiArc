import { createStore } from 'vuex'
import axios from 'axios'

export default createStore({
  state: {
    truckData: [],
    datasets: {},
    datarows: []
  },
  getters: {
    getTruckData (state) {
      return state.truckData
    },
    getDatasets (state) {
      return state.datasets
    },
    getDatarows (state) {
      return state.datarows
    }
  },
  mutations: {
    setTruckData (state, truckData) {
      state.truckData = truckData
    },
    setDatasets (state, datasets) {
      state.datasets = datasets
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
    fetchDatasets (context) {
      axios.get('/datasets').then(response => {
        context.commit('setDatasets', response.data)
      })
    },
    async deleteDataset (context, datasetId) {
      return axios.delete(`/datasets/${datasetId}`)
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
