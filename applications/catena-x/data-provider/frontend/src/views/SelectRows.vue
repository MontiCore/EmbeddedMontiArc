<template>
  <div class="d-flex flex-column align-items-center gap-3">
    <div>
      <div class="text-start">
        <label class="form-label" for="rows">Select rows</label>
        <input class="form-control" type="text" id="rows" v-model="rows"/>
        <div class="form-text text-start">Select rows in the following format: 1-20;31;35-99</div>
      </div>
    </div>
    <div class="d-flex justify-content-center w-100">
      <TruckDataTable style="overflow-x: scroll"/>
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
      rows: ''
    }
  },
  methods: {
    navigateToMetadata () {
      console.log(this.rows)
      this.$router.push(`/create-offer/meta?rows=${this.rows}`)
    }
  }
}
</script>
