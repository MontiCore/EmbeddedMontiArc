<template>
  <div class="d-flex flex-wrap justify-content-center gap-4 mt-3">
    <AddCard />
    <Card v-for="(metadata, id) in offers" :key="id"
          :id="id"
          :provider="metadata.provider"
          :title="metadata.title"
          :description="metadata.description"
          :price="metadata.price"
          :policy="metadata.policy"
    />
  </div>
</template>

<script>
import AddCard from '@/components/AddOffer.vue'
import Card from '@/components/OfferCard.vue'
import 'bootstrap/js/dist/collapse'

export default {
  components: {
    AddCard,
    Card
  },
  data () {
    return {
      showAlert: false
    }
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
