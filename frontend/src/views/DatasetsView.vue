<template>
  <div class="d-flex flex-wrap justify-content-center gap-4 mt-3">
    <AddCard />
    <Card v-for="(metadata, id) in datasets" :key="id"
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
import AddCard from '@/components/AddDataset.vue'
import Card from '@/components/DatasetCard.vue'
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
    datasets () {
      return this.$store.getters.getDatasets
    }
  },
  methods: {
    viewDataset (id) {
      this.$router.push(`/datasets/${id}`)
    },
    alert () {
      this.showAlert = true
    }
  },
  mounted () {
    this.$store.dispatch('fetchDatasets')
  }
}
</script>
