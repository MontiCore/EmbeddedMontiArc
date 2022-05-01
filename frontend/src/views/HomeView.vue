<template>
  <div class="d-flex justify-content-center p-3">
    <DatasetForm />
    <PolicyForm />
  </div>
</template>

<script>
// @ is an alias to /src
import PolicyForm from '@/components/PolicyForm.vue'
import DatasetForm from '@/components/DatasetForm.vue'
import axios from 'axios'

export default {
  name: 'HomeView',
  components: {
    DatasetForm,
    PolicyForm
  },
  data () {
    return {
      truckData: []
    }
  },
  async mounted () {
    console.log('hier bin ich')
    await axios.get('/truckdata').then(response => {
      this.truckData = response.data
    })
  }
}
</script>
