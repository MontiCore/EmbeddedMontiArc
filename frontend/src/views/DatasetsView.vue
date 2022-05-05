<template>
  <div class="d-flex flex-column align-items-center mt-3">
    <div v-if="showAlert" class="alert alert-danger alert-dismissible fade show" role="alert">
      <strong>Policy violation!</strong> Data access not possible!
      <button type="button" class="btn-close" data-bs-dismiss="alert" aria-label="Close"></button>
    </div>
    <div class="d-flex flex-wrap justify-content-center gap-4">
      <AddCard />
      <Card v-for="dataset in datasets" :key="dataset.id"
        :id="dataset.id"
        :provider="dataset.provider"
        :title="dataset.title"
        :description="dataset.description"
        :price="dataset.price"
        @view="viewDataset(dataset.id)"
        @policy-violation="alert"
      />
    </div>
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
