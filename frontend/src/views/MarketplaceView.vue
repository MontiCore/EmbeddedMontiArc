<template>
  <div class="d-flex flex-wrap justify-content-center gap-4 mt-4">
    <Card v-for="offer in offers" :key="offer.id"
      :id="offer.id"
      :provider="offer.provider"
      :title="offer.title"
      :description="offer.description"
      :price="offer.price"
    />
  </div>
</template>

<script>
import Card from '@/components/MarketplaceCard.vue'
import 'bootstrap/js/dist/collapse'

export default {
  components: {
    Card
  },
  computed: {
    offers () {
      return this.$store.getters.getOffers
    }
  },
  mounted () {
    this.$store.dispatch('fetchOffers')
  }
}
</script>
