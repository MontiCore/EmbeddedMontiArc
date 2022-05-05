<template>
  <div ref="card" class="d-flex flex-column shadow text-start gap-2 p-3" style="width: 18rem; border-radius: 0.5rem" :style="collapsedStyle">
    <h5 class="m-0">{{ title }}</h5>
    <div class="d-flex justify-content-between align-items-center">
      <h6 class="card-subtitle text-muted">{{ provider }}</h6>
        <button class="btn p-0"
          type="button" data-bs-toggle="collapse"
          :data-bs-target="'#policies-' + id"
          aria-expanded="false"
          @click="collapsed = !collapsed"
          >
          <div class="d-flex align-items-center" style="color: #036eb8">
            <p class="m-0" style="font-size: 0.8rem">Policies</p>
            <Icon :icon="icon" style="font-size: 1rem" />
          </div>
        </button>
    </div>
    <div :id="'policies-' + id" class="collapse">
      <div class="d-flex flex-column gap-2">
        <div v-if="'usages' in policies" class="d-flex justify-content-between">
          <Icon icon="ic:baseline-replay" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">{{ policies.usages }} {{ usages }}</p>
        </div>
        <div v-if="'time' in policies" class="d-flex justify-content-between">
          <Icon icon="akar-icons:clock" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">{{ policies.time }}</p>
        </div>
        <div v-if="'date' in policies" class="d-flex justify-content-between">
          <Icon icon="akar-icons:calendar" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">{{ policies.date }}</p>
        </div>
        <div v-if="'logging' in policies" class="d-flex justify-content-between">
          <Icon icon="fe:document" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">{{ logging }}</p>
        </div>
        <div v-if="'delete' in policies" class="d-flex justify-content-between">
          <Icon icon="bx:trash" style="font-size: 1.25rem" />
          <p class="m-0 text-nowrap">{{ policies.delete }}</p>
        </div>
      </div>
    </div>
    <p class="description m-0 flex-grow-1">{{ description }}</p>
    <div class="d-flex justify-content-between align-items-center">
      <button class="btn buy-btn fw-bold" @click="viewData">View data</button>
      <p class="fw-bold m-0 fs-5">{{ price.toLocaleString("en-US", { maximumFractionDigits: 2, minimumFractionDigits: 2 }) }} €</p>
    </div>
  </div>
</template>

<script>
import { Icon } from '@iconify/vue'

export default {
  components: {
    Icon
  },
  props: {
    id: {
      type: String,
      required: true
    },
    provider: {
      type: String,
      required: true
    },
    title: {
      type: String,
      required: true
    },
    description: {
      type: String,
      required: true
    },
    price: {
      type: Number,
      required: true
    }
  },
  data () {
    return {
      collapsed: true,
      policies: {
        usages: 5,
        time: '06:00 - 20:00',
        date: '05.04.2022',
        logging: ['local', 'remote'],
        delete: 'delete after'
      }
    }
  },
  computed: {
    usages () {
      return this.policies.usages === 1 ? 'usage' : 'usages'
    },
    logging () {
      return this.policies.logging.join(', ')
    },
    icon () {
      return this.collapsed ? 'ant-design:down-outlined' : 'ant-design:up-outlined'
    },
    collapsedStyle () {
      if (this.collapsed) {
        return {
          'min-height': '18rem',
          'max-height': '18rem'
        }
      } else {
        return {}
      }
    }
  },
  methods: {
    viewData () {
      this.$store.dispatch('fetchDataset', this.id).then(response => {
        this.$store.dispatch('setCurrentDataset', response.data)
        console.log('card: ich hab den ds neu gesetzt')
        console.log(this.$store.getters.getCurrentDataset)
        this.$router.push(`/datasets/${this.id}`)
      })
    }
  }
}
</script>

<style scoped>
.card {
  border-radius: 0.5rem;
}

/* https://stackoverflow.com/questions/3922739/limit-text-length-to-n-lines-using-css */
.description {
   overflow: hidden;
   text-overflow: ellipsis;
   display: -webkit-box;
   -webkit-line-clamp: 6; /* number of lines to show */
           line-clamp: 6;
   -webkit-box-orient: vertical;
}

.buy-btn {
  color: white;
  background-color: #036eb8;
}

.buy-btn:hover, .buy-btn:focus, .buy-btn:active {
  color: white;
  background-color: #005c9e;
}
</style>
