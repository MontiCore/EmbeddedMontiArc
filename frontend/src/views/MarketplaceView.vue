<template>
  <div class="d-flex flex-column align-items-center mt-3">
    <div v-if="alert" class="alert alert-success alert-dismissible fade show" role="alert">
      Succesfully bought dataset <strong>{{ this.lastBought }}</strong>
      <button type="button" class="btn-close" data-bs-dismiss="alert" aria-label="Close"></button>
    </div>
    <div class="d-flex flex-wrap justify-content-center gap-4">
      <Card v-for="offer in offers" :key="offer.id"
        :id="offer.id"
        :provider="offer.provider"
        :title="offer.title"
        :description="offer.description"
        :price="offer.price"
        @buy="showAlert"
      />
    </div>
  </div>
</template>

<script>
import Card from '@/components/OfferCard.vue'
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
  data () {
    return {
      lastBought: '',
      alert: false
    }
  },
  methods: {
    showAlert (datasetTitle) {
      this.lastBought = datasetTitle
      this.alert = true
    }
  },
  mounted () {
    this.$store.dispatch('fetchOffers')
  }
}
</script>
