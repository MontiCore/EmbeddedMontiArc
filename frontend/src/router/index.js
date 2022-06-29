import { createRouter, createWebHashHistory } from 'vue-router'
import SelectRowsView from '../views/SelectRows.vue'
import OffersView from '../views/OffersView.vue'
import OfferMetaView from '../views/OfferMetaView.vue'

const routes = [
  {
    path: '/offers',
    name: 'offers',
    component: OffersView
  },
  {
    path: '/create-offer/rows',
    name: 'selectRowsView',
    component: SelectRowsView
  },
  {
    path: '/create-offer/meta',
    name: 'offerMetaView',
    component: OfferMetaView
  }
]

const router = createRouter({
  history: createWebHashHistory(),
  routes
})

export default router
