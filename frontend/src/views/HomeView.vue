<template>
  <div class="d-flex justify-content-center p-3">
    <!-- <DatasetForm />
    <PolicyForm /> -->
    <TruckDataTable />
  </div>
</template>

<script>
// @ is an alias to /src
// import PolicyForm from '@/components/PolicyForm.vue'
// import DatasetForm from '@/components/DatasetForm.vue'
import TruckDataTable from '@/components/TruckDataTable.vue'

export default {
  name: 'HomeView',
  components: {
    // DatasetForm,
    // PolicyForm,
    TruckDataTable
  }
}
</script>
