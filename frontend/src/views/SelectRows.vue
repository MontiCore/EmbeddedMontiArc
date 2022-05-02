<template>
  <div>
    <div class="d-flex justify-content-center">
      <TruckDataTable :value="rows" @input="rows = $event.target.value"/>
    </div>
    <button class="btn btn-primary" @click="navigateToMetadata">Next</button>
  </div>
</template>

<script>
// @ is an alias to /src
import TruckDataTable from '@/components/TruckDataTable.vue'

export default {
  name: 'HomeView',
  components: {
    TruckDataTable
  },
  data () {
    return {
      rows: '',
      step: 1
    }
  },
  methods: {
    navigateToMetadata () {
      this.$router.push(`/createdataset/meta?file=${this.$route.query.file}&rows=${this.rows}`)
    }
  }
}
</script>
