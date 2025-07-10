import { createRouter, createWebHistory } from 'vue-router'
import DatasetsView from '../views/DatasetsView.vue'
import DatasetDetailView from '../views/DatasetDetailView.vue'
import MarketplaceView from '../views/MarketplaceView.vue'

const routes = [
  {
    path: '/datasets',
    name: 'datasets',
    // route level code-splitting
    // this generates a separate chunk (about.[hash].js) for this route
    // which is lazy-loaded when the route is visited.
    component: DatasetsView
  },
  {
    path: '/datasets/:datasetId',
    name: 'datasetDetail',
    component: DatasetDetailView
  },
  {
    path: '/marketplace',
    name: 'marketplace',
    // route level code-splitting
    // this generates a separate chunk (about.[hash].js) for this route
    // which is lazy-loaded when the route is visited.
    component: MarketplaceView
  }
]

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes
})

export default router
