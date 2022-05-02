import { createRouter, createWebHashHistory } from 'vue-router'
import SelectRowsView from '../views/SelectRows.vue'
import DatasetMetaView from '../views/DatasetMetaView.vue'

const routes = [
  {
    path: '/createdataset/rows',
    name: 'selectRowsView',
    component: SelectRowsView
  },
  {
    path: '/createdataset/meta',
    name: 'datasetMetaView',
    component: DatasetMetaView
  }
]

const router = createRouter({
  history: createWebHashHistory(),
  routes
})

export default router
