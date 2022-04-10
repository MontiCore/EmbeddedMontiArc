<template>
  <div class="d-flex flex-wrap justify-content-center gap-4 mt-4">
    <Card v-for="dataset in datasets" :key="dataset.id"
      :id="dataset.id"
      :provider="dataset.provider"
      :title="dataset.title"
      :description="dataset.description"
      :price="dataset.price"
    />
  </div>
</template>

<script>
import Card from '@/components/DatasetCard.vue'
import 'bootstrap/js/dist/collapse'

export default {
  components: {
    Card
  },
  computed: {
    datasets () {
      return this.$store.getters.getDatasets
    }
  },
  mounted () {
    this.$store.dispatch('fetchDatasets')
  }
}
</script>
